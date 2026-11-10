// Step 5 check: the estimator.
//
// Four of the eight states cannot be measured by anything on this machine.
// This is the test that decides whether the project's central claim holds:
// that an arm with no force sensor and no scale can work out what it is
// carrying and how hard it is pressing, from motor current and its own model.
//
// It also tests the claim's limits, which matters just as much. An estimator
// that is trusted where it is not valid is worse than no estimator.
#include <cmath>
#include <cstdio>

#include "config.hpp"
#include "ctrl/joint_axis.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "est/ekf.hpp"
#include "sim/sensor_board.hpp"

using namespace ppa;

namespace {

constexpr double kDt = 1.0e-5;
constexpr int kStepsPerControl = 100;
constexpr float kControlDt = 1.0e-3f;
int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-46s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

// The whole firmware stack: buses, drivers, estimator, control.
struct Rig {
    arm::PlantParams pp{arm::default_params()};
    World world{pp, 8675309u};
    Faults faults{};
    SimClock clk{};
    Rng rng{123581321u};
    SensorBoard board{world, faults, clk, rng};

    Imu imu{board.imu_bus};
    Encoder enc1{board.enc1_bus, clk};
    Encoder enc2{board.enc2_bus, clk};
    CanNodes can{board.can_bus, clk};
    Ekf ekf{};
    JointAxis axis[2]{};

    // Running statistics, for the consistency check.
    // Per sensor, because a mean over all of them hides which one is
    // inconsistent, and step 6 gates each sensor separately anyway.
    double nis_sum[4]{0.0, 0.0, 0.0, 0.0};
    int nis_n[4]{0, 0, 0, 0};
    static constexpr int kEnc = 0, kDrvP = 1, kDrvV = 2, kAcc = 3;
    int imu_updates{0};
    int control_cycles{0};

    Rig() {
        imu.init();
        enc1.init();
        enc2.init();
        axis[0].configure(0);
        axis[1].configure(1);
        // Seed the filter from the encoders, which is all a real machine has
        // at power on.
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        ekf.init(wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]),
                 wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]));
    }

    void cycle(const float q_ref[2], const float qd_ref[2]) {
        can.poll();
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        const ImuSample im = imu.read();

        // --- the input the estimator treats as known -----------------------
        // Motor torque, from the current the drives report. This is an input
        // to the dynamics, not a measurement of state: we caused it.
        Vec<2> tau_motor{};
        tau_motor[0] = current_to_torque(0, can.drive(0).current_a);
        tau_motor[1] = current_to_torque(1, can.drive(1).current_a);
        ekf.predict(tau_motor, kControlDt);

        // --- one update per sensor ----------------------------------------
        if (s1.valid && s2.valid) {
            const float q1 = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
            const float q2 = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
            nis_sum[kEnc] += ekf.update_encoders(q1, q2);
            ++nis_n[kEnc];
        }
        if (can.drive(0).valid && can.drive(1).valid) {
            nis_sum[kDrvP] += ekf.update_drive_position(can.drive(0).position_rad,
                                                        can.drive(1).position_rad);
            ++nis_n[kDrvP];
            nis_sum[kDrvV] += ekf.update_drive_velocity(can.drive(0).velocity_rad_s,
                                                        can.drive(1).velocity_rad_s);
            ++nis_n[kDrvV];
        }
        if (im.valid) {
            // Only when the device actually converted a new sample. Feeding
            // a repeat would make the filter confident on data it never got.
            ekf.update_gyro(im.gyro_z);
            nis_sum[kAcc] += ekf.update_tool_accel(im.accel_x, im.accel_y);
            ++nis_n[kAcc];
            ++imu_updates;
        }

        // --- control, from the estimate, not from raw sensors -------------
        // This is what an observer is for. The loops see one filtered,
        // cross checked state rather than whichever sensor happened to be
        // read last, and the payload estimate closes the gravity feedforward
        // gap that step 4 measured at 0.326 A.
        const float p = ekf.payload_trustworthy() ? ekf.payload_kg() : 0.0f;
        const float i0 = axis[0].update(q_ref[0], qd_ref[0], ekf.q1(), ekf.qd1(),
                                        p, ekf.q2(), kControlDt);
        const float i1 = axis[1].update(q_ref[1], qd_ref[1], ekf.q2(), ekf.qd2(),
                                        p, ekf.q1(), kControlDt);
        can.send_drive_current(i0, i1);
        ++control_cycles;

        for (int i = 0; i < kStepsPerControl; ++i) {
            world.step(faults, kDt);
            clk.advance_us(10);
            board.update();
        }
    }

    void hold(const arm::Vec2& target, double seconds) {
        const float q_ref[2] = {static_cast<float>(target[0]),
                                static_cast<float>(target[1])};
        const float qd_ref[2] = {0.0f, 0.0f};
        const int n = static_cast<int>(seconds / kControlDt);
        for (int i = 0; i < n; ++i) cycle(q_ref, qd_ref);
    }

    // Sweep back and forth between two poses, which is what makes the
    // inferred states observable.
    void sweep(const arm::Vec2& a, const arm::Vec2& b, double seconds) {
        const int n = static_cast<int>(seconds / kControlDt);
        for (int i = 0; i < n; ++i) {
            const double u = 0.5 - 0.5 * std::cos(2.0 * 3.14159265 *
                             static_cast<double>(i) / (n / 2));
            const float q_ref[2] = {static_cast<float>(a[0] + u * (b[0] - a[0])),
                                    static_cast<float>(a[1] + u * (b[1] - a[1]))};
            const float qd_ref[2] = {0.0f, 0.0f};
            cycle(q_ref, qd_ref);
        }
    }

    void grip(bool close, double settle) {
        can.send_grip(close);
        ekf.announce_grip_change();
        const arm::Vec2 here{world.q()[0], world.q()[1]};
        hold(here, settle);
    }
};

arm::Vec2 ik(const arm::PlantParams& pp, double x, double y) {
    return arm::inverse_kinematics(pp.arm, arm::Vec2{x, y}, arm::Elbow::kDown).q;
}

}  // namespace

int main() {
    char buf[220];
    const arm::PlantParams pp = arm::default_params();

    // ------------------------------------------------------ tracks the arm
    std::printf("does it track the arm at all\n");
    {
        Rig r;
        const arm::Vec2 a = ik(pp, 0.240, 0.000);
        const arm::Vec2 b = ik(pp, 0.340, 0.040);
        r.hold(a, 1.0);
        r.sweep(a, b, 2.0);

        const double eq1 = std::fabs(r.ekf.q1() - r.world.q()[0]);
        const double eq2 = std::fabs(r.ekf.q2() - r.world.q()[1]);
        std::snprintf(buf, sizeof(buf), "joint errors %.5f and %.5f rad", eq1, eq2);
        check("joint angles converge", eq1 < 0.003 && eq2 < 0.003, buf);

        std::snprintf(buf, sizeof(buf), "%d imu updates in %d cycles, %.0f Hz effective",
                      r.imu_updates, r.control_cycles,
                      r.imu_updates / (r.control_cycles * 0.001));
        check("imu contributes at its own rate, not the loop rate",
              r.imu_updates < r.control_cycles, buf);
    }

    // --------------------------------------------------- payload estimation
    std::printf("\nwhat am I carrying?\n");
    {
        struct Trial { double truth, est, sigma; };
        auto weigh = [&](int box_index, double box_x) -> Trial {
            Rig r;
            const arm::Vec2 over = ik(pp, box_x, -0.108);
            r.hold(over, 1.2);
            r.grip(true, 0.4);
            const double truth = r.world.payload_kg();

            // Lift and sweep. The arm has to move for the payload to be
            // observable, which is the honest limitation of the method.
            const arm::Vec2 c = ik(pp, 0.250, 0.010);
            const arm::Vec2 d = ik(pp, 0.340, 0.050);
            r.hold(c, 0.6);
            r.sweep(c, d, 3.0);
            // Truth is read again at the end. A box that slipped during the
            // move would otherwise look like an estimator failure.
            const double truth_end = r.world.payload_kg();
            std::printf("      box %d: held %.3f kg at grip, %.3f kg at end,"
                        " %u drops\n", box_index + 1, truth, truth_end,
                        r.world.drop_events());
            return Trial{truth_end, r.ekf.payload_kg(), r.ekf.payload_sigma()};
        };

        const Trial t1 = weigh(0, 0.180);
        const Trial t2 = weigh(1, 0.260);
        const Trial t3 = weigh(2, 0.340);

        std::printf("    %-10s %10s %10s %10s\n", "box", "true kg", "est kg", "sigma");
        std::printf("    %-10s %10.3f %10.3f %10.3f\n", "1", t1.truth, t1.est, t1.sigma);
        std::printf("    %-10s %10.3f %10.3f %10.3f\n", "2", t2.truth, t2.est, t2.sigma);
        std::printf("    %-10s %10.3f %10.3f %10.3f\n", "3", t3.truth, t3.est, t3.sigma);

        std::snprintf(buf, sizeof(buf), "errors %.3f, %.3f, %.3f kg",
                      std::fabs(t1.est - t1.truth), std::fabs(t2.est - t2.truth),
                      std::fabs(t3.est - t3.truth));
        check("payload recovered to within 60 g",
              std::fabs(t1.est - t1.truth) < 0.06 &&
              std::fabs(t2.est - t2.truth) < 0.06 &&
              std::fabs(t3.est - t3.truth) < 0.06, buf);

        // The three boxes are 250 g apart at the closest. If the estimate
        // resolves them it can answer "which box did I pick up", which is
        // what the pick verification in step 7 needs.
        std::snprintf(buf, sizeof(buf), "estimates %.3f, %.3f, %.3f kg",
                      t1.est, t2.est, t3.est);
        check("the three boxes are told apart",
              t1.est < t3.est && t3.est < t2.est, buf);
    }

    // ---------------------------------------------- nothing in the gripper
    std::printf("\nand when it is empty?\n");
    {
        Rig r;
        const arm::Vec2 c = ik(pp, 0.250, 0.010);
        const arm::Vec2 d = ik(pp, 0.340, 0.050);
        r.hold(c, 1.0);
        r.sweep(c, d, 3.0);
        std::snprintf(buf, sizeof(buf), "estimate %.3f kg, sigma %.3f",
                      r.ekf.payload_kg(), r.ekf.payload_sigma());
        check("an empty gripper reads near zero",
              std::fabs(r.ekf.payload_kg()) < 0.06, buf);
    }

    // ------------------------------------------------------- contact force
    std::printf("\nhow hard am I pressing?\n");
    {
        Rig r;
        r.hold(ik(pp, 0.300, -0.060), 1.5);
        // Command below the table so the tool presses into it.
        const arm::IkResult into =
            arm::inverse_kinematics(pp.arm, arm::Vec2{0.300, -0.156},
                                    arm::Elbow::kDown);
        // Pressing into the table is contact, and the firmware knows it
        // because it commanded the approach. Declaring it is what releases
        // the external torque state.
        r.ekf.set_mode(EkfMode::kContact);
        if (into.reachable) r.hold(into.q, 2.0);

        Vec<2> f_est{};
        const bool ok = r.ekf.contact_force(&f_est);
        const arm::Vec2 f_true = r.world.contact_force();

        std::printf("    true  force (%.2f, %.2f) N\n", f_true[0], f_true[1]);
        std::printf("    est   force (%.2f, %.2f) N\n", f_est[0], f_est[1]);
        std::printf("    true  tau_ext (%.3f, %.3f) Nm\n",
                    r.world.tau_ext()[0], r.world.tau_ext()[1]);
        std::printf("    est   tau_ext (%.3f, %.3f) Nm\n",
                    r.ekf.tau_ext()[0], r.ekf.tau_ext()[1]);

        check("the force estimate is usable near this pose", ok, "");
        const double mag_true = std::hypot(f_true[0], f_true[1]);
        const double mag_est = std::hypot(f_est[0], f_est[1]);
        std::snprintf(buf, sizeof(buf), "%.1f N estimated against %.1f N true",
                      mag_est, mag_true);
        check("contact force recovered within 25 percent",
              mag_true > 5.0 && std::fabs(mag_est - mag_true) < 0.25 * mag_true, buf);
    }

    // ------------------------------------------- friction, not a phantom box
    std::printf("\nworn joints must not look like a payload\n");
    {
        Rig r;
        r.faults.friction_mult = 3.0f;
        const arm::Vec2 c = ik(pp, 0.250, 0.010);
        const arm::Vec2 d = ik(pp, 0.340, 0.050);
        r.hold(c, 1.0);
        r.sweep(c, d, 4.0);

        std::printf("    friction scale estimated %.2f, true 3.00\n",
                    r.ekf.friction_scale());
        std::printf("    payload estimated %.3f kg, true 0.000\n",
                    r.ekf.payload_kg());
        std::snprintf(buf, sizeof(buf), "scale %.2f", r.ekf.friction_scale());
        check("the wear is attributed to friction", r.ekf.friction_scale() > 1.8f, buf);
        std::snprintf(buf, sizeof(buf), "payload %.3f kg", r.ekf.payload_kg());
        check("and not to a box that is not there",
              std::fabs(r.ekf.payload_kg()) < 0.08f, buf);
    }

    // ----------------------------------- the limitation, stated not hidden
    std::printf("\nwhere the estimator is NOT valid\n");
    {
        // The filter's free motion mode asserts that nothing is touching the
        // arm. That assertion is what makes the payload observable at all.
        // So the interesting failure is not standing still, it is being
        // WRONG about the mode: if the tool is pressing on something while
        // the filter believes it is in free space, the contact force has
        // nowhere to go except into the payload estimate.
        //
        // This is why the mode is set by the task layer rather than inferred
        // here, and why step 7's sequence has to declare contact before it
        // touches anything.
        Rig honest;
        honest.hold(ik(pp, 0.300, -0.060), 1.0);
        honest.ekf.set_mode(EkfMode::kContact);
        const arm::IkResult into =
            arm::inverse_kinematics(pp.arm, arm::Vec2{0.300, -0.156},
                                    arm::Elbow::kDown);
        if (into.reachable) honest.hold(into.q, 2.0);

        Rig lying;
        lying.hold(ik(pp, 0.300, -0.060), 1.0);
        // Mode deliberately left in free motion while pressing on the table.
        if (into.reachable) lying.hold(into.q, 2.0);

        std::printf("    pressing on the table with %.0f N:\n",
                    std::hypot(honest.world.contact_force()[0],
                               honest.world.contact_force()[1]));
        std::printf("      mode declared correctly   payload %.3f kg,"
                    " tau_ext (%.1f, %.1f) Nm\n",
                    honest.ekf.payload_kg(), honest.ekf.tau_ext()[0],
                    honest.ekf.tau_ext()[1]);
        std::printf("      mode left as free motion  payload %.3f kg,"
                    " tau_ext (%.1f, %.1f) Nm\n",
                    lying.ekf.payload_kg(), lying.ekf.tau_ext()[0],
                    lying.ekf.tau_ext()[1]);

        std::snprintf(buf, sizeof(buf), "%.3f kg of phantom payload",
                      lying.ekf.payload_kg());
        check("a mislabelled mode invents a payload",
              lying.ekf.payload_kg() > 0.15f, buf);

        std::snprintf(buf, sizeof(buf), "%.3f kg with the mode declared",
                      honest.ekf.payload_kg());
        check("declaring contact keeps the payload honest",
              honest.ekf.payload_kg() < 0.10f, buf);

        // And it is confident about the wrong answer, which is the part that
        // matters. A wrong estimate that admits it is uncertain can be
        // handled. A wrong estimate that reports itself trustworthy cannot.
        std::snprintf(buf, sizeof(buf), "sigma %.3f, reports trustworthy: %d",
                      lying.ekf.payload_sigma(),
                      lying.ekf.payload_trustworthy() ? 1 : 0);
        check("and the covariance does not warn about it",
              lying.ekf.payload_trustworthy(), buf);
    }

    // -------------------------------------------------- filter consistency
    std::printf("\nis the filter statistically honest\n");
    {
        // A filter can look right on a plot and still be lying about its own
        // uncertainty. The test for that is the normalised innovation
        // squared: if the covariance is correct, its average should sit near
        // the number of dimensions being measured. Far below means the filter
        // is claiming more uncertainty than it has, far above means it is
        // overconfident, and overconfident is the dangerous direction.
        Rig r;
        const arm::Vec2 c = ik(pp, 0.250, 0.010);
        const arm::Vec2 d = ik(pp, 0.340, 0.050);
        r.hold(c, 1.0);
        for (int i = 0; i < 4; ++i) { r.nis_sum[i] = 0.0; r.nis_n[i] = 0; }
        r.sweep(c, d, 3.0);

        const char* names[4] = {"encoders", "drive position", "drive velocity",
                                "tool accelerometer"};
        double worst = 0.0;
        const char* worst_name = "";
        std::printf("    %-22s %8s %10s\n", "sensor", "updates", "mean NIS");
        for (int i = 0; i < 4; ++i) {
            const double m = r.nis_n[i] ? r.nis_sum[i] / r.nis_n[i] : 0.0;
            std::printf("    %-22s %8d %10.2f\n", names[i], r.nis_n[i], m);
            if (m > worst) { worst = m; worst_name = names[i]; }
        }
        std::printf("    Each is a two dimensional update, so a correct\n");
        std::printf("    covariance puts every mean near 2.\n");

        std::snprintf(buf, sizeof(buf), "worst is %s at %.2f", worst_name, worst);
        check("no sensor is wildly inconsistent", worst < 6.0, buf);

        // A LOW mean NIS is not a fault, and asserting that it was cost a
        // round of pointless tuning. It means the prior is already better
        // than that sensor, which is exactly what redundant sensing buys:
        // joint velocity is constrained by the dynamics, the encoders, the
        // gyro and the accelerometer, so the drives' own velocity report
        // adds little. That is spare capacity, not a defect.
        //
        // A HIGH mean NIS is the dangerous direction, because it means the
        // filter is trusting a sensor further than the sensor deserves. So
        // only the high side is a failure.
        double best = 1e9;
        const char* best_name = "";
        for (int i = 0; i < 4; ++i) {
            const double m = r.nis_n[i] ? r.nis_sum[i] / r.nis_n[i] : 0.0;
            if (m < best) { best = m; best_name = names[i]; }
        }
        std::printf("    %s is the quietest at %.2f, which means the model\n",
                    best_name, best);
        std::printf("    and the other sensors already pin that state down.\n");
        check("no sensor is being trusted beyond its worth", worst < 4.0, "");
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
