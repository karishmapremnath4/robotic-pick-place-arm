// The firmware, on the target.
//
// Every layer above the four bus interfaces is the same code the simulation
// runs: the same drivers, the same filter, the same control loops, the same
// fault logic, the same sequence. Only what sits beneath those interfaces
// differs, and that is the whole argument for having had them.
//
// What is NOT here, deliberately: the cell. No World, no device models, no
// fault injection. Those are the test bench and they stay on the host.
#include "config.hpp"
#include "ctrl/joint_axis.hpp"
#include "diag/safety.hpp"
#include "diag/sensor_health.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "drivers/laser.hpp"
#include "est/ekf.hpp"
#include "rtos/scheduler.hpp"
#include "task/sequence.hpp"

extern "C" unsigned long cpu_hz(void);
extern "C" unsigned long stack_guard_intact_bytes(void);

namespace ppa {

void init_buses();
I2cBus& imu_bus();
SpiBus& encoder1_bus();
SpiBus& encoder2_bus();
CanBus& can_bus();
UartBus& laser_bus();
Clock& system_clock();

namespace {

constexpr float kControlDt = 1.0e-3f;

// Everything in static storage. There is no heap on this part and _sbrk
// always fails, so anything that tried to allocate would fail to link.
Imu g_imu(imu_bus());
Encoder g_enc1(encoder1_bus(), system_clock());
Encoder g_enc2(encoder2_bus(), system_clock());
CanNodes g_can(can_bus(), system_clock());
LaserRanger g_laser(laser_bus(), system_clock());
Ekf g_ekf;
SensorHealth g_health;
SafetySupervisor g_safety;
Sequence g_seq;
JointAxis g_axis[2];
Scheduler g_sched;

bool g_grip_cmd = false;

// --- the tasks ------------------------------------------------------------
//
// Five jobs at four rates. Rate monotonic: the fastest job gets the highest
// priority, which for a run to completion scheduler is the ordering that
// bounds the blocking any task can suffer.

struct Shared {
    float q[2];
    float qd[2];
    Vec<2> tau_motor;
    float force_n;
    float raw_force_n;
    float raw_force_y;
    bool enc_valid;
    bool imu_valid;
};
Shared g_s;

void task_sensors(void*) {
    g_can.poll();
    g_laser.poll();
    const EncoderSample s1 = g_enc1.read();
    const EncoderSample s2 = g_enc2.read();
    g_s.enc_valid = s1.valid && s2.valid;
    if (g_s.enc_valid) {
        g_s.q[0] = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
        g_s.q[1] = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
    }
    g_s.qd[0] = g_can.drive(0).velocity_rad_s;
    g_s.qd[1] = g_can.drive(1).velocity_rad_s;
    g_s.tau_motor[0] = current_to_torque(0, g_can.drive(0).current_a);
    g_s.tau_motor[1] = current_to_torque(1, g_can.drive(1).current_a);
}

void task_estimate(void*) {
    const TimeUs now = system_clock().now_us();
    g_ekf.predict(g_s.tau_motor, kControlDt);

    if (!g_s.enc_valid) g_health.report_hard_fault(SensorId::kEncoders, now);
    else g_health.clear_hard_fault(SensorId::kEncoders);
    if (g_laser.timed_out()) g_health.report_hard_fault(SensorId::kLaser, now);
    else g_health.clear_hard_fault(SensorId::kLaser);

    if (g_s.enc_valid) {
        const float nis = g_ekf.update_encoders(
            g_s.q[0], g_s.q[1], g_health.should_fuse(SensorId::kEncoders));
        g_health.report_nis(SensorId::kEncoders, nis, 2, now);
    }
    if (g_can.drive(0).valid && g_can.drive(1).valid) {
        g_health.report_nis(SensorId::kDrivePosition,
            g_ekf.update_drive_position(g_can.drive(0).position_rad,
                                        g_can.drive(1).position_rad,
                g_health.should_fuse(SensorId::kDrivePosition)), 2, now);
        g_health.report_nis(SensorId::kDriveVelocity,
            g_ekf.update_drive_velocity(g_can.drive(0).velocity_rad_s,
                                        g_can.drive(1).velocity_rad_s,
                g_health.should_fuse(SensorId::kDriveVelocity)), 2, now);
        if (g_s.enc_valid)
            g_health.report_channel_pair(g_s.q[0], g_s.q[1],
                                         g_can.drive(0).position_rad,
                                         g_can.drive(1).position_rad,
                                         g_ekf.qd1(), g_ekf.qd2(),
                                         kControlDt, now);
    }
    const ImuSample im = g_imu.read();
    if (im.valid) {
        const bool f = g_health.should_fuse(SensorId::kToolImu);
        g_ekf.update_gyro(im.gyro_z, f);
        g_health.report_nis(SensorId::kToolImu,
                            g_ekf.update_tool_accel(im.accel_x, im.accel_y, f),
                            2, now);
    }

    Vec<2> f{};
    g_s.force_n = g_ekf.contact_force(&f)
                      ? __builtin_sqrtf(f[0] * f[0] + f[1] * f[1])
                      : 0.0f;
    const RawContact rc = raw_contact(
        g_ekf.q1(), g_ekf.q2(), g_ekf.qd1(), g_ekf.qd2(),
        g_s.tau_motor[0], g_s.tau_motor[1],
        g_ekf.payload_trustworthy() ? g_ekf.payload_kg() : 0.0f,
        g_ekf.friction_scale());
    g_s.raw_force_n = rc.valid ? rc.force_n : 0.0f;
    g_s.raw_force_y = rc.valid ? rc.force_y_n : 0.0f;
}

void task_safety(void*) {
    SafetyInputs si{};
    si.drive_node_lost[0] = g_can.node_lost(kNodeDrive1);
    si.drive_node_lost[1] = g_can.node_lost(kNodeDrive2);
    si.gripper_node_lost = g_can.node_lost(kNodeGripper);
    si.channels_disagree = g_health.channels_disagree();
    si.contact_force_n = g_s.force_n;
    si.in_contact_mode = (g_ekf.mode() == EkfMode::kContact);
    si.payload_kg = g_ekf.payload_kg();
    si.gripper_holding = (g_can.gripper().state == GripReport::kHolding);
    si.grip_commanded_closed = g_grip_cmd;
    si.payload_sigma_kg = g_ekf.payload_sigma();
    si.tau_sigma_nm = g_ekf.tau_ext_sigma(0);
    si.payload_trustworthy = g_ekf.payload_trustworthy();
    g_safety.update(si, g_health, system_clock().now_us());
}

void task_control(void*) {
    TaskInputs ti{};
    ti.q1 = g_ekf.q1();
    ti.q2 = g_ekf.q2();
    ti.payload_kg = g_ekf.payload_kg();
    ti.payload_trustworthy = g_ekf.payload_trustworthy();
    ti.contact_force_n = g_s.force_n;
    ti.raw_contact_force_n = g_s.raw_force_n;
    ti.raw_force_y_n = g_s.raw_force_y;
    ti.gripper_holding = (g_can.gripper().state == GripReport::kHolding);
    ti.clamp_n = g_can.gripper().clamp_n;
    ti.safety = g_safety.state();
    const TaskOutputs to = g_seq.step(ti);

    g_ekf.set_mode(to.ekf_mode);
    if (to.grip_close != g_grip_cmd) {
        g_grip_cmd = to.grip_close;
        g_can.send_grip(g_grip_cmd);
        g_ekf.announce_grip_change();
    }

    const float scale = g_safety.speed_scale();
    const float p = g_ekf.payload_trustworthy() ? g_ekf.payload_kg() : 0.0f;
    float i0 = 0.0f, i1 = 0.0f;
    if (g_safety.motion_allowed() && to.pose_valid) {
        i0 = g_axis[0].update(to.q_ref[0], 0.0f, g_ekf.q1(), g_ekf.qd1(), p,
                              g_ekf.q2(), kControlDt) * scale;
        i1 = g_axis[1].update(to.q_ref[1], 0.0f, g_ekf.q2(), g_ekf.qd2(), p,
                              g_ekf.q1(), kControlDt) * scale;
    }
    g_can.send_drive_current(i0, i1);
}

// Telemetry out of the serial port. Deliberately the lowest priority: if the
// processor ever runs short, the thing to give up is talking about the work,
// never the work.
void task_telemetry(void*) {
    char line[72];
    int n = 0;
    auto put = [&](const char* s) { while (*s && n < 70) line[n++] = *s++; };
    auto num = [&](int v) {
        char b[12];
        int k = 0;
        if (v < 0) { put("-"); v = -v; }
        do { b[k++] = static_cast<char>('0' + v % 10); v /= 10; } while (v && k < 11);
        while (k) line[n++] = b[--k];
    };
    put("t=");       num(static_cast<int>(system_clock().now_us() / 1000));
    put(" st=");     num(static_cast<int>(g_seq.state()));
    put(" sf=");     num(static_cast<int>(g_safety.state()));
    put(" pay=");    num(static_cast<int>(g_ekf.payload_kg() * 1000.0f));
    put(" f=");      num(static_cast<int>(g_s.force_n));
    put(" miss=");   num(static_cast<int>(g_sched.total_deadline_misses()));
    put(" guard=");  num(static_cast<int>(stack_guard_intact_bytes()));
    line[n++] = '\r';
    line[n++] = '\n';
    laser_bus().write(reinterpret_cast<const u8*>(line), static_cast<u8>(n));
}

}  // namespace
}  // namespace ppa

int main(void)
{
    using namespace ppa;

    init_buses();

    g_imu.init();
    g_enc1.init();
    g_enc2.init();
    g_axis[0].configure(0);
    g_axis[1].configure(1);

    TaskConfig tc{};
    g_seq.configure(tc);

    const EncoderSample s1 = g_enc1.read();
    const EncoderSample s2 = g_enc2.read();
    g_ekf.init(wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]),
               wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]));

    g_sched.add("sensors",   1000, 0, 1000, task_sensors,   nullptr);
    g_sched.add("estimate",  1000, 1, 1000, task_estimate,  nullptr);
    g_sched.add("safety",    1000, 2, 1000, task_safety,    nullptr);
    g_sched.add("control",   1000, 3, 1000, task_control,   nullptr);
    g_sched.add("telemetry", 20000, 4, 20000, task_telemetry, nullptr);

    for (;;) {
        g_sched.tick(system_clock().now_us());
    }
}
