// Step 7 check: the job, running by itself.
//
// The question is not whether the arm can pick up a box when everything
// works. It is whether it notices when the pick silently fails, when the box
// falls off halfway, when it grabbed the wrong one, and whether it refuses
// to carry on in each case.
#include <cmath>
#include <cstdio>

#include "config.hpp"
#include "ctrl/joint_axis.hpp"
#include "diag/safety.hpp"
#include "diag/sensor_health.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "drivers/laser.hpp"
#include "est/ekf.hpp"
#include "sim/sensor_board.hpp"
#include "task/sequence.hpp"

using namespace ppa;

namespace {

constexpr double kDt = 1.0e-5;
constexpr int kStepsPerControl = 100;
constexpr float kControlDt = 1.0e-3f;
int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-48s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

struct Rig {
    arm::PlantParams pp{arm::default_params()};
    World world{pp, 20261007u};
    Faults faults{};
    SimClock clk{};
    Rng rng{31415926u};
    SensorBoard board{world, faults, clk, rng};

    Imu imu{board.imu_bus};
    Encoder enc1{board.enc1_bus, clk};
    Encoder enc2{board.enc2_bus, clk};
    CanNodes can{board.can_bus, clk};
    LaserRanger laser{board.laser_bus, clk};
    Ekf ekf{};
    SensorHealth health{};
    SafetySupervisor safety{};
    Sequence seq{};
    JointAxis axis[2]{};

    bool grip_cmd{false};
    bool abort_noted{false};
    TaskEvent log[32]{};
    int n_log{0};

    Rig() {
        imu.init();
        enc1.init();
        enc2.init();
        axis[0].configure(0);
        axis[1].configure(1);
        TaskConfig tc{};
        tc.table_y = static_cast<float>(world.layout().table_y);
        seq.configure(tc);
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        ekf.init(wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]),
                 wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]));
    }

    void cycle() {
        const TimeUs now = clk.now_us();
        can.poll();
        laser.poll();
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        const ImuSample im = imu.read();

        Vec<2> tau_motor{};
        tau_motor[0] = current_to_torque(0, can.drive(0).current_a);
        tau_motor[1] = current_to_torque(1, can.drive(1).current_a);
        ekf.predict(tau_motor, kControlDt);

        if (!s1.valid || !s2.valid) health.report_hard_fault(SensorId::kEncoders, now);
        else health.clear_hard_fault(SensorId::kEncoders);
        if (laser.timed_out()) health.report_hard_fault(SensorId::kLaser, now);
        else health.clear_hard_fault(SensorId::kLaser);

        float eq1 = 0.0f, eq2 = 0.0f;
        if (s1.valid && s2.valid) {
            eq1 = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
            eq2 = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
            health.report_nis(SensorId::kEncoders,
                              ekf.update_encoders(eq1, eq2,
                                  health.should_fuse(SensorId::kEncoders)), 2, now);
        }
        if (can.drive(0).valid && can.drive(1).valid) {
            health.report_nis(SensorId::kDrivePosition,
                ekf.update_drive_position(can.drive(0).position_rad,
                                          can.drive(1).position_rad,
                    health.should_fuse(SensorId::kDrivePosition)), 2, now);
            health.report_nis(SensorId::kDriveVelocity,
                ekf.update_drive_velocity(can.drive(0).velocity_rad_s,
                                          can.drive(1).velocity_rad_s,
                    health.should_fuse(SensorId::kDriveVelocity)), 2, now);
            if (s1.valid && s2.valid)
                health.report_channel_pair(eq1, eq2, can.drive(0).position_rad,
                                           can.drive(1).position_rad, ekf.qd1(),
                                           ekf.qd2(), kControlDt, now);
        }
        if (im.valid) {
            const bool f = health.should_fuse(SensorId::kToolImu);
            ekf.update_gyro(im.gyro_z, f);
            health.report_nis(SensorId::kToolImu,
                              ekf.update_tool_accel(im.accel_x, im.accel_y, f), 2, now);
        }

        Vec<2> force{};
        const bool have_force = ekf.contact_force(&force);
        const float force_mag =
            have_force ? std::sqrt(force[0] * force[0] + force[1] * force[1]) : 0.0f;

        SafetyInputs si{};
        si.drive_node_lost[0] = can.node_lost(kNodeDrive1);
        si.drive_node_lost[1] = can.node_lost(kNodeDrive2);
        si.gripper_node_lost = can.node_lost(kNodeGripper);
        si.channels_disagree = health.channels_disagree();
        si.contact_force_n = force_mag;
        si.in_contact_mode = (ekf.mode() == EkfMode::kContact);
        si.payload_kg = ekf.payload_kg();
        si.gripper_holding = (can.gripper().state == GripReport::kHolding);
        si.grip_commanded_closed = grip_cmd;
        si.payload_sigma_kg = ekf.payload_sigma();
        si.tau_sigma_nm = ekf.tau_ext_sigma(0);
        si.payload_trustworthy = ekf.payload_trustworthy();
        safety.update(si, health, now);

        // --- the sequence ------------------------------------------------
        TaskInputs ti{};
        ti.q1 = ekf.q1();
        ti.q2 = ekf.q2();
        ti.payload_kg = ekf.payload_kg();
        ti.payload_trustworthy = ekf.payload_trustworthy();
        ti.contact_force_n = force_mag;
        const RawContact rc = raw_contact(
            ekf.q1(), ekf.q2(), ekf.qd1(), ekf.qd2(),
            tau_motor[0], tau_motor[1],
            ekf.payload_trustworthy() ? ekf.payload_kg() : 0.0f,
            ekf.friction_scale());
        ti.raw_contact_force_n = rc.valid ? rc.force_n : 0.0f;
        ti.raw_force_y_n = rc.valid ? rc.force_y_n : 0.0f;
        ti.gripper_holding = si.gripper_holding;
        ti.clamp_n = can.gripper().clamp_n;
        ti.safety = safety.state();
        const TaskOutputs to = seq.step(ti);

        // The sequence declares the estimator's regime. See sequence.hpp for
        // why this cannot be inferred inside the filter.
        ekf.set_mode(to.ekf_mode);

        if (to.grip_close != grip_cmd) {
            grip_cmd = to.grip_close;
            can.send_grip(grip_cmd);
            // A discontinuity the filter cannot predict is coming.
            ekf.announce_grip_change();
        }

        for (TaskEvent e = seq.take_event(); e != TaskEvent::kNone;
             e = seq.take_event()) {
            if (n_log < 32) log[n_log++] = e;
            if (e == TaskEvent::kAbortedBySafety && !abort_noted) {
                abort_noted = true;
                std::printf("      [aborted by %s: %s]\n",
                            SafetySupervisor::state_name(safety.state()),
                            SafetySupervisor::reason_name(safety.reason()));
            }
        }

        const float scale = safety.speed_scale();
        const float p = ekf.payload_trustworthy() ? ekf.payload_kg() : 0.0f;
        float i0 = 0.0f, i1 = 0.0f;
        if (safety.motion_allowed() && to.pose_valid) {
            i0 = axis[0].update(to.q_ref[0], 0.0f, ekf.q1(), ekf.qd1(), p,
                                ekf.q2(), kControlDt) * scale;
            i1 = axis[1].update(to.q_ref[1], 0.0f, ekf.q2(), ekf.qd2(), p,
                                ekf.q1(), kControlDt) * scale;
        }
        can.send_drive_current(i0, i1);

        for (int i = 0; i < kStepsPerControl; ++i) {
            world.step(faults, kDt);
            clk.advance_us(10);
            board.update();
        }
    }

    void run(double seconds) {
        const int n = static_cast<int>(seconds / kControlDt);
        for (int i = 0; i < n; ++i) cycle();
    }

    // Run until the sequence finishes or fails, returning seconds taken.
    double run_job(double max_seconds) {
        const int n = static_cast<int>(max_seconds / kControlDt);
        for (int i = 0; i < n; ++i) {
            cycle();
            if (seq.state() == TaskState::kDone || seq.state() == TaskState::kFailed)
                return i * kControlDt;
        }
        return -1.0;
    }

    bool saw(TaskEvent e) const {
        for (int i = 0; i < n_log; ++i) if (log[i] == e) return true;
        return false;
    }
    void print_events(const char* indent) const {
        for (int i = 0; i < n_log; ++i)
            std::printf("%s%s\n", indent, Sequence::event_name(log[i]));
    }
};

}  // namespace

int main() {
    char buf[220];

    // ----------------------------------------------------- a complete job
    std::printf("a job that works\n");
    {
        Rig r;
        r.run(1.0);  // settle, let the filter converge
        Job j{};
        j.pick_x = 0.260f;   // box 2, half a kilogram
        j.place_x = 0.120f;
        const double box_x_before = r.world.box(1).x;
        r.seq.start(j);
        const double t = r.run_job(25.0);
        r.run(0.5);

        std::printf("    finished in %.2f s, state %s\n", t,
                    Sequence::state_name(r.seq.state()));
        r.print_events("      ");
        std::printf("    measured %.3f kg, true %.3f kg\n", r.seq.last_measured_kg(), 0.5);
        std::printf("    box 2 moved from x %.3f to x %.3f\n", box_x_before,
                    r.world.box(1).x);

        std::snprintf(buf, sizeof(buf), "state %s after %.2f s",
                      Sequence::state_name(r.seq.state()), t);
        check("the job completes", r.seq.state() == TaskState::kDone, buf);
        std::snprintf(buf, sizeof(buf), "%.3f kg against 0.500 kg",
                      r.seq.last_measured_kg());
        check("it weighed the box on the way", 
              std::fabs(r.seq.last_measured_kg() - 0.5) < 0.08, buf);
        std::snprintf(buf, sizeof(buf), "now at x %.3f, asked for 0.120",
                      r.world.box(1).x);
        check("the box actually moved there",
              std::fabs(r.world.box(1).x - 0.120) < 0.02, buf);
        check("and it reported placing it", r.saw(TaskEvent::kPlaced), "");
    }

    // ------------------------------------------------ nothing there to pick
    std::printf("\npicking from an empty spot\n");
    {
        Rig r;
        r.run(1.0);
        Job j{};
        j.pick_x = 0.215f;   // between boxes, bare table
        j.place_x = 0.120f;
        r.seq.start(j);
        const double t = r.run_job(30.0);
        std::printf("    finished in %.2f s, state %s\n", t,
                    Sequence::state_name(r.seq.state()));
        r.print_events("      ");
        check("the job fails rather than pretending",
              r.seq.state() == TaskState::kFailed, "");
        check("and says the gripper came up empty",
              r.saw(TaskEvent::kPickFailedEmpty), "");
    }

    // ----------------------------------------------- the box falls off
    std::printf("\nthe box falls off halfway across\n");
    {
        Rig r;
        r.run(1.0);
        Job j{};
        j.pick_x = 0.260f;
        j.place_x = 0.120f;
        r.seq.start(j);

        // Wait until it is carrying the box, then make the grip leak.
        bool leaked = false;
        for (int i = 0; i < 25000; ++i) {
            r.cycle();
            // Injected as soon as the arm has the box clear of the table.
            // The clamp decays from 40 N at 60 N/s and a 0.5 kg box needs
            // 16 N of it, so the box lets go about 400 ms later, which puts
            // the drop squarely in the middle of the traverse.
            //
            // An earlier version injected this 200 cycles into the traverse
            // instead, and the box then held on until the place descent. The
            // arm descended onto the box it had just dropped, felt contact,
            // released nothing, and reported the job placed.
            if (!leaked && r.seq.state() == TaskState::kLift) {
                r.faults.grip_leak = true;
                leaked = true;
            }
            if (r.seq.state() == TaskState::kDone ||
                r.seq.state() == TaskState::kFailed) break;
        }
        std::printf("    state %s\n", Sequence::state_name(r.seq.state()));
        r.print_events("      ");
        std::snprintf(buf, sizeof(buf), "%u drop events in the world",
                      r.world.drop_events());
        check("the drop is detected in transit",
              r.saw(TaskEvent::kBoxDroppedInTransit), buf);
        check("and the job stops", r.seq.state() == TaskState::kFailed, "");
    }

    // --------------------------------------------------- the wrong box
    std::printf("\nthe wrong box\n");
    {
        Rig r;
        r.run(1.0);
        Job j{};
        j.pick_x = 0.260f;      // box 2 is 0.500 kg
        j.place_x = 0.120f;
        j.expect_kg = 0.250f;   // but the job expected box 1's mass
        r.seq.start(j);
        const double t = r.run_job(25.0);
        std::printf("    finished in %.2f s, measured %.3f kg, expected %.3f kg\n",
                    t, r.seq.last_measured_kg(), 0.25);
        r.print_events("      ");
        check("a mass mismatch stops the job", r.saw(TaskEvent::kWrongMass), "");
        check("rather than placing the wrong item",
              r.seq.state() == TaskState::kFailed, "");
    }

    // ------------------------------------------- safety outranks the job
    std::printf("\nsafety outranks the sequence\n");
    {
        Rig r;
        r.run(1.0);
        Job j{};
        j.pick_x = 0.260f;
        j.place_x = 0.120f;
        r.seq.start(j);
        r.run(2.0);
        const TaskState before = r.seq.state();
        r.faults.can_silent_node = kNodeDrive2;
        r.run(1.0);

        std::printf("    was %s, now %s, safety %s\n",
                    Sequence::state_name(before),
                    Sequence::state_name(r.seq.state()),
                    SafetySupervisor::state_name(r.safety.state()));
        check("a drive loss aborts the job",
              r.seq.state() == TaskState::kFailed, "");
        check("and it says why", r.saw(TaskEvent::kAbortedBySafety), "");
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
