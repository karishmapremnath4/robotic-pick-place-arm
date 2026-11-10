// Step 6 check: does the arm notice when a sensor lies to it, and does it do
// the right thing about it?
//
// Two numbers matter for every fault: whether it is caught at all, and how
// long that takes. The second is the one that gets quoted, so it is measured
// here rather than estimated.
//
// A third number matters just as much and is easy to forget: how often a
// healthy arm is wrongly declared broken. A fault detector with a false
// alarm rate is worse than none, because the operator learns to ignore it.
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

// The whole firmware: buses, drivers, estimator, health, safety, control.
struct Rig {
    arm::PlantParams pp{arm::default_params()};
    World world{pp, 4815162342u};
    Faults faults{};
    SimClock clk{};
    Rng rng{1123581321u};
    SensorBoard board{world, faults, clk, rng};

    Imu imu{board.imu_bus};
    Encoder enc1{board.enc1_bus, clk};
    Encoder enc2{board.enc2_bus, clk};
    CanNodes can{board.can_bus, clk};
    LaserRanger laser{board.laser_bus, clk};
    Ekf ekf{};
    SensorHealth health{};
    SafetySupervisor safety{};
    JointAxis axis[2]{};

    u32 control_cycles{0};

    Rig() {
        imu.init();
        enc1.init();
        enc2.init();
        axis[0].configure(0);
        axis[1].configure(1);
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        ekf.init(wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]),
                 wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]));
    }

    void cycle(const float q_ref[2]) {
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

        // --- hard faults, straight from the drivers -----------------------
        // These are not statistical. The device said it was broken.
        if (!s1.valid || !s2.valid)
            health.report_hard_fault(SensorId::kEncoders, now);
        else
            health.clear_hard_fault(SensorId::kEncoders);

        if (!im.valid && imu.stale_reads() == last_stale_)
            health.report_hard_fault(SensorId::kToolImu, now);
        last_stale_ = imu.stale_reads();

        if (laser.timed_out())
            health.report_hard_fault(SensorId::kLaser, now);
        else
            health.clear_hard_fault(SensorId::kLaser);

        // --- statistical monitoring, and conditional fusion ---------------
        // Every sensor is monitored every cycle, whether or not it is fused.
        // A faulted sensor's disagreement is then measured against a filter
        // it is not influencing, which is how it earns its way back.
        float enc_q1 = 0.0f, enc_q2 = 0.0f;
        if (s1.valid && s2.valid) {
            enc_q1 = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
            enc_q2 = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
            const bool fuse = health.should_fuse(SensorId::kEncoders);
            const float nis = ekf.update_encoders(enc_q1, enc_q2, fuse);
            health.report_nis(SensorId::kEncoders, nis, 2, now);
        }

        if (can.drive(0).valid && can.drive(1).valid) {
            const bool fuse_p = health.should_fuse(SensorId::kDrivePosition);
            const float nis_p = ekf.update_drive_position(
                can.drive(0).position_rad, can.drive(1).position_rad, fuse_p);
            health.report_nis(SensorId::kDrivePosition, nis_p, 2, now);

            const bool fuse_v = health.should_fuse(SensorId::kDriveVelocity);
            const float nis_v = ekf.update_drive_velocity(
                can.drive(0).velocity_rad_s, can.drive(1).velocity_rad_s, fuse_v);
            health.report_nis(SensorId::kDriveVelocity, nis_v, 2, now);

            // Model free cross check of the two position channels. Faster
            // than the NIS and needs no filter at all, but it can only say
            // that one of the two is wrong, never which.
            if (s1.valid && s2.valid)
                health.report_channel_pair(enc_q1, enc_q2,
                                           can.drive(0).position_rad,
                                           can.drive(1).position_rad,
                                           ekf.qd1(), ekf.qd2(), kControlDt, now);
        }

        if (im.valid) {
            const bool fuse = health.should_fuse(SensorId::kToolImu);
            ekf.update_gyro(im.gyro_z, fuse);
            const float nis = ekf.update_tool_accel(im.accel_x, im.accel_y, fuse);
            health.report_nis(SensorId::kToolImu, nis, 2, now);
        }

        // --- supervisory safety -------------------------------------------
        Vec<2> force{};
        const bool have_force = ekf.contact_force(&force);
        SafetyInputs in{};
        in.drive_node_lost[0] = can.node_lost(kNodeDrive1);
        in.drive_node_lost[1] = can.node_lost(kNodeDrive2);
        in.gripper_node_lost = can.node_lost(kNodeGripper);
        in.channels_disagree = health.channels_disagree();
        in.contact_force_n =
            have_force ? std::sqrt(force[0] * force[0] + force[1] * force[1]) : 0.0f;
        in.in_contact_mode = (ekf.mode() == EkfMode::kContact);
        in.payload_kg = ekf.payload_kg();
        in.gripper_holding = (can.gripper().state == GripReport::kHolding);
        in.payload_sigma_kg = ekf.payload_sigma();
        in.tau_sigma_nm = ekf.tau_ext_sigma(0);
        in.payload_trustworthy = ekf.payload_trustworthy();
        safety.update(in, health, now);

        // --- control, scaled by whatever authority safety allows ----------
        const float scale = safety.speed_scale();
        const float p = ekf.payload_trustworthy() ? ekf.payload_kg() : 0.0f;
        float i0 = 0.0f, i1 = 0.0f;
        if (safety.motion_allowed()) {
            i0 = axis[0].update(q_ref[0], 0.0f, ekf.q1(), ekf.qd1(), p,
                                ekf.q2(), kControlDt) * scale;
            i1 = axis[1].update(q_ref[1], 0.0f, ekf.q2(), ekf.qd2(), p,
                                ekf.q1(), kControlDt) * scale;
        }
        can.send_drive_current(i0, i1);
        ++control_cycles;

        for (int i = 0; i < kStepsPerControl; ++i) {
            world.step(faults, kDt);
            clk.advance_us(10);
            board.update();
        }
    }

    void hold(const arm::Vec2& t, double seconds) {
        const float q_ref[2] = {static_cast<float>(t[0]), static_cast<float>(t[1])};
        const int n = static_cast<int>(seconds / kControlDt);
        for (int i = 0; i < n; ++i) cycle(q_ref);
    }

    // Run until a predicate fires, returning how many milliseconds it took.
    // -1 means it never fired.
    template <typename Pred>
    int run_until(const arm::Vec2& t, double max_seconds, Pred p) {
        const float q_ref[2] = {static_cast<float>(t[0]), static_cast<float>(t[1])};
        const int n = static_cast<int>(max_seconds / kControlDt);
        for (int i = 0; i < n; ++i) {
            cycle(q_ref);
            if (p()) return i;
        }
        return -1;
    }

    u32 last_stale_{0};
};

arm::Vec2 ik(const arm::PlantParams& pp, double x, double y) {
    return arm::inverse_kinematics(pp.arm, arm::Vec2{x, y}, arm::Elbow::kDown).q;
}

struct Latency { const char* fault; int ms; const char* response; };
Latency g_lat[8];
int g_nlat = 0;
void record(const char* f, int ms, const char* r) {
    if (g_nlat < 8) g_lat[g_nlat++] = Latency{f, ms, r};
}

}  // namespace

int main() {
    char buf[220];
    const arm::PlantParams pp = arm::default_params();

    // ------------------------------------------------- no false alarms
    std::printf("a healthy arm is not declared broken\n");
    {
        Rig r;
        const arm::Vec2 a = ik(pp, 0.380, 0.060);
        const arm::Vec2 b = ik(pp, 0.180, -0.040);
        r.hold(ik(pp, 0.250, 0.000), 1.5);
        for (int i = 0; i < 3; ++i) { r.hold(a, 1.0); r.hold(b, 1.0); }

        std::printf("    %d control cycles of hard step moves\n", r.control_cycles);
        std::printf("    %-18s %10s %10s %12s\n", "sensor", "worst NIS", "rejected", "health");
        for (int i = 0; i < kNumSensors; ++i) {
            const SensorId s = static_cast<SensorId>(i);
            const SensorStatus& st = r.health.status(s);
            std::printf("    %-18s %10.2f %10u %12s\n", SensorHealth::name(s),
                        st.worst_nis, st.total_rejected,
                        SensorHealth::health_name(st.health));
        }
        std::snprintf(buf, sizeof(buf), "%d sensors faulted", r.health.faulted_count());
        check("no sensor is falsely faulted", r.health.faulted_count() == 0, buf);
        std::snprintf(buf, sizeof(buf), "state %s",
                      SafetySupervisor::state_name(r.safety.state()));
        check("the supervisor stays in NORMAL",
              r.safety.state() == SafetyState::kNormal, buf);
    }

    // ------------------------------------------- a sensor that lies quietly
    std::printf("\na frozen encoder, which looks perfectly healthy\n");
    {
        Rig r;
        r.hold(ik(pp, 0.250, 0.000), 1.5);
        r.faults.enc_stuck_joint = 1;

        const int ms = r.run_until(ik(pp, 0.350, 0.040), 2.0, [&] {
            return r.health.health(SensorId::kEncoders) == Health::kFaulted ||
                   r.health.channels_disagree();
        });

        std::printf("    channel gap %.4f rad, encoder health %s\n",
                    r.health.channel_gap_rad(),
                    SensorHealth::health_name(r.health.health(SensorId::kEncoders)));
        std::snprintf(buf, sizeof(buf), "%d ms", ms);
        check("detected", ms >= 0 && ms < 400, buf);
        record("encoder frozen", ms, "degrade, drop the channel");

        // The arm must keep working on the healthy channel rather than stop.
        r.hold(ik(pp, 0.300, 0.020), 1.5);
        std::printf("    verdict: %s | encoders %s | drive position %s\n",
                    r.health.has_verdict()
                        ? SensorHealth::name(r.health.arbitration_verdict())
                        : "none",
                    SensorHealth::health_name(r.health.health(SensorId::kEncoders)),
                    SensorHealth::health_name(r.health.health(SensorId::kDrivePosition)));
        std::printf("    encoder passes since fault: %u, recover threshold 500\n",
                    r.health.status(SensorId::kEncoders).consecutive_passes);
        const double err = std::fabs(r.ekf.q1() - r.world.q()[0]);
        std::snprintf(buf, sizeof(buf), "estimate still within %.4f rad of truth", err);
        check("the estimate survives on the other channel", err < 0.02, buf);
        std::snprintf(buf, sizeof(buf), "state %s",
                      SafetySupervisor::state_name(r.safety.state()));
        check("and the arm degrades rather than stopping",
              r.safety.state() == SafetyState::kDegraded, buf);
    }

    // -------------------------------------------- faults the driver reports
    std::printf("\nfaults the devices admit to\n");
    {
        Rig r;
        r.hold(ik(pp, 0.250, 0.000), 1.5);
        r.faults.enc_error_frames = true;
        int ms = r.run_until(ik(pp, 0.250, 0.000), 0.5, [&] {
            return r.health.health(SensorId::kEncoders) == Health::kFaulted;
        });
        std::snprintf(buf, sizeof(buf), "%d ms", ms);
        check("encoder error flag is immediate", ms >= 0 && ms <= 2, buf);
        record("encoder error flag", ms, "degrade");
        r.faults.enc_error_frames = false;

        Rig r2;
        r2.hold(ik(pp, 0.250, 0.000), 1.5);
        r2.faults.laser_silent = true;
        ms = r2.run_until(ik(pp, 0.250, 0.000), 0.5, [&] {
            return r2.health.health(SensorId::kLaser) == Health::kFaulted;
        });
        std::snprintf(buf, sizeof(buf), "%d ms, timeout is %.0f ms", ms,
                      kLaserTimeoutUs / 1000.0);
        check("a silent laser is caught by its timeout", ms >= 0 && ms < 120, buf);
        record("laser silent", ms, "degrade");
    }

    // ------------------------------------------------- a drive stops talking
    std::printf("\na motor drive stops answering\n");
    {
        Rig r;
        r.hold(ik(pp, 0.250, 0.000), 1.5);
        r.faults.can_silent_node = kNodeDrive2;
        const int ms = r.run_until(ik(pp, 0.250, 0.000), 1.0, [&] {
            return r.safety.state() == SafetyState::kSafeStop;
        });
        std::snprintf(buf, sizeof(buf), "%d ms, heartbeat timeout is %.0f ms", ms,
                      kHeartbeatTimeoutUs / 1000.0);
        check("trips to SAFE STOP", ms >= 0 && ms < 500, buf);
        std::snprintf(buf, sizeof(buf), "reason: %s",
                      SafetySupervisor::reason_name(r.safety.reason()));
        check("and names the right reason",
              r.safety.reason() == TripReason::kDriveNodeLost, buf);
        record("drive node lost", ms, "SAFE STOP");

        // The latch: clearing the cause must not restart the arm.
        r.faults.can_silent_node = 0;
        r.hold(ik(pp, 0.250, 0.000), 0.6);
        check("the stop latches even once the fault clears",
              r.safety.state() == SafetyState::kSafeStop, "");
        check("and motion stays blocked", !r.safety.motion_allowed(), "");

        // What the arm was doing while stopped matters here. With the drives
        // commanded to zero it sags under gravity, so by the time anyone
        // acknowledges, the tool is resting on the table. That is genuine
        // contact, and the collision detector is right to see it.
        std::printf("    while stopped the tool sagged to y = %.4f m,"
                    " table at %.4f m\n",
                    r.world.tip()[1], r.world.layout().table_y);

        r.safety.acknowledge();
        std::snprintf(buf, sizeof(buf), "state %s immediately after",
                      SafetySupervisor::state_name(r.safety.state()));
        check("an acknowledgement clears the latch",
              r.safety.motion_allowed(), buf);

        // And this is the operational finding: acknowledging is not the same
        // as being ready to run. The arm is now leaning on something, so the
        // first thing it does is trip again. A real restart needs a recovery
        // move that lifts clear before normal operation resumes, which is
        // step 7's job, not the supervisor's.
        r.hold(ik(pp, 0.250, 0.000), 0.3);
        std::printf("    resuming straight into the table: %s, %s\n",
                    SafetySupervisor::state_name(r.safety.state()),
                    SafetySupervisor::reason_name(r.safety.reason()));
        check("and resuming without lifting clear trips again, correctly",
              !r.safety.motion_allowed(), "the arm is leaning on the table");
    }

    // --------------------------- a collision, with no force sensor involved
    std::printf("\nhitting something, detected without a force sensor\n");
    {
        Rig r;
        r.faults.obstacle_present = true;
        r.hold(ik(pp, 0.250, 0.050), 1.5);

        // Drive straight at the obstacle while the filter believes it is in
        // free space. Nothing in the machine measures force.
        const arm::Vec2 target = ik(pp, r.world.layout().obstacle_x,
                                    r.world.layout().obstacle_y);
        const int ms = r.run_until(target, 1.5, [&] {
            return r.safety.state() == SafetyState::kSafeStop;
        });

        Vec<2> f{};
        r.ekf.contact_force(&f);
        std::printf("    true contact force          %.1f N\n",
                    std::hypot(r.world.contact_force()[0], r.world.contact_force()[1]));
        std::printf("    estimated force             %.1f N  (free motion mode\n",
                    std::sqrt(f[0] * f[0] + f[1] * f[1]));
        std::printf("                                       cannot report one)\n");
        std::printf("    phantom payload             %.3f kg, gripper open\n",
                    r.ekf.payload_kg());
        std::printf("    that is the detectable signal: the jaws are open, so no\n");
        std::printf("    payload is physically possible at all.\n");
        std::snprintf(buf, sizeof(buf), "%d ms", ms);
        check("collision trips the arm", ms >= 0 && ms < 600, buf);
        std::snprintf(buf, sizeof(buf), "reason: %s",
                      SafetySupervisor::reason_name(r.safety.reason()));
        check("identified as unexpected contact",
              r.safety.reason() == TripReason::kUnexpectedContact, buf);
        record("collision in free motion", ms, "SAFE STOP");
    }

    // ------------------------------------------------- detection sensitivity
    std::printf("\nhow small a fault gets through\n");
    {
        // The conservative tuning that removed the false alarms also costs
        // sensitivity. This measures that cost instead of ignoring it: how
        // large does a stuck encoder's error have to grow before the
        // model free channel comparison notices?
        std::printf("    the channel comparison trips at a %.0f mrad gap,\n", 20.0);
        std::printf("    which at the %.0f rad/s joint limit is %.1f ms of motion.\n",
                    8.0, 1000.0 * 0.020 / 8.0);
        std::printf("    A stuck encoder on a stationary arm therefore goes\n");
        std::printf("    undetected until the arm next moves, which is correct:\n");
        std::printf("    a stationary arm has no evidence to find it with.\n");

        Rig r;
        r.hold(ik(pp, 0.250, 0.000), 1.5);
        r.faults.enc_stuck_joint = 1;
        r.hold(ik(pp, 0.250, 0.000), 1.0);  // frozen, but the arm is parked
        const bool caught_while_still = r.health.channels_disagree();
        std::snprintf(buf, sizeof(buf), "channel gap only %.4f rad while parked",
                      r.health.channel_gap_rad());
        check("a frozen encoder is invisible on a parked arm",
              !caught_while_still, buf);

        const int ms = r.run_until(ik(pp, 0.340, 0.040), 1.0,
                                   [&] { return r.health.channels_disagree(); });
        std::snprintf(buf, sizeof(buf), "found %d ms after the arm started moving", ms);
        check("and found as soon as it moves", ms >= 0 && ms < 300, buf);
    }

    // --------------------------------------------------------- the summary
    std::printf("\nmeasured detection latency\n");
    std::printf("    %-26s %10s   %s\n", "fault", "detected", "response");
    for (int i = 0; i < g_nlat; ++i)
        std::printf("    %-26s %7d ms   %s\n", g_lat[i].fault, g_lat[i].ms,
                    g_lat[i].response);

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
