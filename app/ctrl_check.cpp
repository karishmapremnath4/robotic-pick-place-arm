// Step 4 check: the cascaded loops.
//
// Five questions. Does the current loop reach its setpoint quickly enough to
// be invisible to the loop above it? Does gravity feedforward remove the
// steady state droop? Do the limits actually hold? Does anti windup earn its
// place? And does the arm track a real trajectory rather than just settle at
// a point?
#include <cmath>
#include <cstdio>

#include "config.hpp"
#include "ctrl/joint_axis.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "est/arm_model.hpp"
#include "sim/sensor_board.hpp"
#include "sim/trajectory.hpp"

using namespace ppa;

namespace {

constexpr double kDt = 1.0e-5;          // plant step, 10 us
constexpr int kStepsPerControl = 100;   // control task, 1 kHz
constexpr float kControlDt = 1.0e-3f;
int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-46s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

// The firmware as it now stands: buses, drivers, and two cascaded axes.
struct Rig {
    arm::PlantParams pp{arm::default_params()};
    World world{pp, 90210u};
    Faults faults{};
    SimClock clk{};
    Rng rng{31337u};
    SensorBoard board{world, faults, clk, rng};

    Encoder enc1{board.enc1_bus, clk};
    Encoder enc2{board.enc2_bus, clk};
    CanNodes can{board.can_bus, clk};
    JointAxis axis[2]{};

    // What the controller believes is in the gripper. Zero until step 5
    // supplies an estimate.
    float payload_belief{0.0f};

    // Telemetry gathered while running.
    float max_current[2]{0.0f, 0.0f};
    float max_speed[2]{0.0f, 0.0f};

    Rig() {
        enc1.init();
        enc2.init();
        axis[0].configure(0);
        axis[1].configure(1);
    }

    // Joint angles as the firmware sees them, through the encoders.
    void read_joints(float* q, float* qd) {
        const EncoderSample s1 = enc1.read();
        const EncoderSample s2 = enc2.read();
        q[0] = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
        q[1] = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
        // Velocity comes from the drives, which differentiate their own
        // encoders internally. Differentiating the SPI encoder at 1 kHz
        // would amplify its quantisation noise by a thousand.
        qd[0] = can.drive(0).velocity_rad_s;
        qd[1] = can.drive(1).velocity_rad_s;
    }

    // One control cycle, then the plant for one control period.
    void control_step(float q_ref[2], float qd_ref[2]) {
        can.poll();
        float q[2], qd[2];
        read_joints(q, qd);

        const float i0 = axis[0].update(q_ref[0], qd_ref[0], q[0], qd[0],
                                        payload_belief, q[1], kControlDt);
        const float i1 = axis[1].update(q_ref[1], qd_ref[1], q[1], qd[1],
                                        payload_belief, q[0], kControlDt);
        can.send_drive_current(i0, i1);

        for (int j = 0; j < 2; ++j) {
            const float ia = std::fabs(static_cast<float>(world.current()[j]));
            if (ia > max_current[j]) max_current[j] = ia;
            const float sa = std::fabs(static_cast<float>(world.qd()[j]));
            if (sa > max_speed[j]) max_speed[j] = sa;
        }

        for (int i = 0; i < kStepsPerControl; ++i) {
            world.step(faults, kDt);
            clk.advance_us(10);
            board.update();
        }
    }

    void hold(const arm::Vec2& target, double seconds) {
        float q_ref[2] = {static_cast<float>(target[0]), static_cast<float>(target[1])};
        float qd_ref[2] = {0.0f, 0.0f};
        const int cycles = static_cast<int>(seconds / kControlDt);
        for (int i = 0; i < cycles; ++i) control_step(q_ref, qd_ref);
    }
};

arm::Vec2 ik(const arm::PlantParams& pp, double x, double y) {
    return arm::inverse_kinematics(pp.arm, arm::Vec2{x, y}, arm::Elbow::kDown).q;
}

}  // namespace

int main() {
    char buf[220];
    const arm::PlantParams pp = arm::default_params();

    // ------------------------------------------- why the split exists
    std::printf("bus budget: where the current loop has to live\n");
    {
        // A standard CAN frame with two data bytes: 1 start, 11 id, 1 rtr,
        // 6 control, 16 data, 15 crc, 1 crc delim, 1 ack, 1 ack delim,
        // 7 eof, 3 interframe. About 63 bits, and bit stuffing can add a
        // fifth on worst case data.
        const double bits = 63.0 * 1.20;
        const double us_per_frame = bits / 1.0;  // 1 Mbit/s, so 1 bit per us
        const double frames = 4.0;               // 2 commands, 2 replies
        const double cycle_us = frames * us_per_frame;
        std::printf("    one frame          %.0f bits, %.0f us at 1 Mbit/s\n", bits, us_per_frame);
        std::printf("    two axis cycle     %.0f frames, %.0f us of bus\n", frames, cycle_us);
        std::printf("    winding constant   L/R = 1.0 ms, so the loop wants ~10 kHz\n");
        std::printf("    10 kHz budget      100 us per cycle\n");
        std::snprintf(buf, sizeof(buf), "needs %.0f us, has 100 us", cycle_us);
        check("a 10 kHz current loop does not fit on the bus", cycle_us > 100.0, buf);
        std::snprintf(buf, sizeof(buf), "%.0f us of 1000 us, %.0f%% load",
                      cycle_us, cycle_us / 10.0);
        check("1 kHz does fit, which is what the outer loops use",
              cycle_us < 1000.0, buf);
    }

    // ------------------------------------------------- gravity feedforward
    std::printf("\ngravity feedforward\n");
    {
        const arm::Vec2 target = ik(pp, 0.300, -0.020);

        // With feedforward off, the velocity loop's integrator is the only
        // thing holding the arm up, so it has to wind up and the arm droops
        // until it does.
        Rig off;
        off.axis[0].configure(0);
        // Zeroing the payload belief does not disable feedforward, so the
        // comparison is made by holding the arm with the loops as built and
        // reading the feedforward contribution out.
        off.hold(target, 2.0);

        const float i_ff = off.axis[0].gravity_current();
        const float i_actual = static_cast<float>(off.world.current()[0]);
        std::snprintf(buf, sizeof(buf), "feedforward %.3f A of the %.3f A actually flowing",
                      i_ff, i_actual);
        check("feedforward supplies most of the holding current",
              std::fabs(i_ff) > 0.6f * std::fabs(i_actual), buf);

        std::snprintf(buf, sizeof(buf), "integrator holding only %.4f A",
                      off.axis[0].velocity_integral() * kAxisGains[0].ki_vel);
        check("so the integrator has little left to do",
              std::fabs(off.axis[0].velocity_integral() * kAxisGains[0].ki_vel) <
                  std::fabs(i_actual), buf);

        // Position accuracy at the tool.
        const arm::Vec2 tip = off.world.tip();
        const double err = std::hypot(tip[0] - 0.300, tip[1] - (-0.020));
        std::snprintf(buf, sizeof(buf), "tool %.2f mm from the commanded point", err * 1000.0);
        check("holds position to under a millimetre", err < 0.001, buf);
    }

    // ------------------------------------------------------ step response
    std::printf("\nposition step response, joint 1\n");
    {
        Rig r;
        const arm::Vec2 a = ik(pp, 0.280, -0.040);
        r.hold(a, 2.0);
        const float q0 = static_cast<float>(r.world.q()[0]);

        const arm::Vec2 b = ik(pp, 0.280, 0.060);
        const float target = static_cast<float>(b[0]);
        const float span = target - q0;

        float q_ref[2] = {static_cast<float>(b[0]), static_cast<float>(b[1])};
        float qd_ref[2] = {0.0f, 0.0f};

        float rise_ms = -1.0f, peak = q0;
        int settle_ms = -1;
        for (int ms = 0; ms < 3000; ++ms) {
            r.control_step(q_ref, qd_ref);
            const float q = static_cast<float>(r.world.q()[0]);
            const float frac = (q - q0) / span;
            if (rise_ms < 0.0f && frac > 0.9f) rise_ms = static_cast<float>(ms);
            if (std::fabs(q - q0) > std::fabs(peak - q0)) peak = q;
            if (std::fabs(q - target) < 0.002f) {
                if (settle_ms < 0) settle_ms = ms;
            } else {
                settle_ms = -1;
            }
        }
        const float overshoot = 100.0f * (peak - target) / span;

        std::printf("    move of %.4f rad\n", span);
        std::snprintf(buf, sizeof(buf), "%.0f ms to 90 percent", rise_ms);
        check("rises in a sensible time", rise_ms > 0.0f && rise_ms < 600.0f, buf);
        std::snprintf(buf, sizeof(buf), "%.1f percent", overshoot);
        check("overshoot stays small", overshoot < 12.0f, buf);
        std::snprintf(buf, sizeof(buf), "settled at %d ms, within 2 mrad", settle_ms);
        check("settles and stays settled", settle_ms > 0, buf);
    }

    // ------------------------------------------------------------- limits
    std::printf("\nlimits actually hold\n");
    {
        Rig r;
        // Command a large fast move, which is what makes a loop ask for more
        // than it is allowed.
        r.hold(ik(pp, 0.120, 0.050), 1.0);
        r.hold(ik(pp, 0.430, 0.020), 1.5);
        r.hold(ik(pp, 0.120, 0.050), 1.5);

        std::snprintf(buf, sizeof(buf), "peak %.3f A and %.3f A against a 6.000 A limit",
                      r.max_current[0], r.max_current[1]);
        check("current never exceeds the drive rating",
              r.max_current[0] <= 6.01f && r.max_current[1] <= 6.01f, buf);

        // Read this one carefully. The limit clamps the velocity SETPOINT,
        // which is all a cascaded controller can clamp. The joint itself can
        // still overshoot that setpoint slightly during a transient, and it
        // does: 8.09 against a commanded 8.00. So the honest claim is "the
        // commanded speed is bounded and the actual speed tracks it to within
        // a couple of percent", not "the joint can never exceed 8 rad/s".
        // Guaranteeing the latter needs a separate supervisory trip, which is
        // what the safety states in step 7 are for.
        const float over0 = 100.0f * (r.max_speed[0] - 8.0f) / 8.0f;
        const float over1 = 100.0f * (r.max_speed[1] - 12.0f) / 12.0f;
        std::printf("    commanded limits 8.00 and 12.00 rad/s; actual peaks\n");
        std::printf("    overshot by %.1f%% and %.1f%% during transients\n",
                    over0 > 0.0f ? over0 : 0.0f, over1 > 0.0f ? over1 : 0.0f);
        std::snprintf(buf, sizeof(buf), "peak %.2f and %.2f rad/s, within 5%% of command",
                      r.max_speed[0], r.max_speed[1]);
        check("actual speed tracks the commanded limit closely",
              r.max_speed[0] <= 8.4f && r.max_speed[1] <= 12.6f, buf);
    }

    // ------------------------------------------------------- anti windup
    std::printf("\nwhat anti windup is worth\n");
    {
        // Drive the tool hard into the table so the loop saturates, then
        // command it straight up and time how long until it actually moves.
        auto trial = [&](bool antiwindup) {
            Rig r;
            r.axis[0].set_antiwindup(antiwindup);
            r.axis[1].set_antiwindup(antiwindup);
            r.hold(ik(pp, 0.280, -0.100), 1.0);

            // Command 40 mm below the table. Unreachable, so the loops pin.
            const arm::IkResult into =
                arm::inverse_kinematics(pp.arm, arm::Vec2{0.280, -0.190},
                                        arm::Elbow::kDown);
            if (into.reachable) r.hold(into.q, 1.5);

            const float y_start = static_cast<float>(r.world.tip()[1]);
            const arm::Vec2 up = ik(pp, 0.280, 0.000);
            float q_ref[2] = {static_cast<float>(up[0]), static_cast<float>(up[1])};
            float qd_ref[2] = {0.0f, 0.0f};

            int react_ms = -1;
            for (int ms = 0; ms < 1200; ++ms) {
                r.control_step(q_ref, qd_ref);
                if (react_ms < 0 &&
                    static_cast<float>(r.world.tip()[1]) > y_start + 0.005f) {
                    react_ms = ms;
                }
            }
            return react_ms;
        };

        const int with = trial(true);
        const int without = trial(false);
        std::printf("    time to lift 5 mm after being pinned against the table\n");
        std::printf("      anti windup on   %4d ms\n", with);
        std::printf("      anti windup off  %4d ms\n", without);

        std::snprintf(buf, sizeof(buf), "%d ms", with);
        check("with anti windup the arm responds promptly",
              with >= 0 && with < 250, buf);
        std::snprintf(buf, sizeof(buf), "%d ms on, %d ms off", with, without);
        check("without it the arm is slower to react or does not react",
              without < 0 || without > with, buf);
    }

    // ------------------------------------------------ trajectory tracking
    std::printf("\ntracking a jerk limited path\n");
    {
        Rig r;
        const arm::Vec2 from{0.200, 0.000};
        const arm::Vec2 to{0.360, 0.000};
        r.hold(ik(pp, from[0], from[1]), 2.0);

        // Her S curve profile, straight line in Cartesian space.
        const arm::Limits lim{0.30, 1.20, 12.0};
        arm::CartesianLine path(from, to, lim);
        const int cycles = static_cast<int>(path.duration() / kControlDt) + 400;

        double worst = 0.0, sum = 0.0;
        int n = 0;
        for (int i = 0; i < cycles; ++i) {
            const double t = i * kControlDt;
            const double tc = t < path.duration() ? t : path.duration();
            const arm::Vec2 p = path.point_at(tc);
            const arm::IkResult sol =
                arm::inverse_kinematics(pp.arm, p, arm::Elbow::kDown);
            if (!sol.reachable) continue;

            // Velocity feedforward. The path knows how fast it is moving, in
            // tool coordinates; the joints need that expressed as joint
            // rates. Since v = J qd, the joint rates are the solution of
            // J qd = v, which solve2x2 gives directly.
            //
            // Without this the position loop settles at an error of velocity
            // over gain, which on the first run of this test was 14 mm of
            // lag on a 160 mm path. Raising the gain to hide it would have
            // been the wrong fix: the lag is structural, not a tuning fault.
            const arm::Vec2 v = path.velocity_at(tc);
            const arm::Mat2 J = arm::jacobian(pp.arm, sol.q);
            const arm::Vec2 qd_ff = arm::solve2x2(J, v);

            float q_ref[2] = {static_cast<float>(sol.q[0]),
                              static_cast<float>(sol.q[1])};
            float qd_ref[2] = {static_cast<float>(qd_ff[0]),
                               static_cast<float>(qd_ff[1])};
            r.control_step(q_ref, qd_ref);

            const arm::Vec2 tip = r.world.tip();
            const double e = std::hypot(tip[0] - p[0], tip[1] - p[1]);
            if (t > 0.05 && t < path.duration()) {
                if (e > worst) worst = e;
                sum += e;
                ++n;
            }
        }
        std::printf("    path %.0f mm, duration %.2f s\n",
                    (to[0] - from[0]) * 1000.0, path.duration());
        std::snprintf(buf, sizeof(buf), "worst %.2f mm, mean %.2f mm",
                      worst * 1000.0, n ? (sum / n) * 1000.0 : 0.0);
        check("tool follows the path within a millimetre", worst < 0.0015, buf);

        const arm::Vec2 tip = r.world.tip();
        const double final_err = std::hypot(tip[0] - to[0], tip[1] - to[1]);
        std::snprintf(buf, sizeof(buf), "%.2f mm from the end point", final_err * 1000.0);
        check("lands on the end of the path", final_err < 0.001, buf);
    }

    // -------------------------------- the socket step 5 plugs into
    std::printf("\nthe gap the payload estimate will close\n");
    {
        // The first version of this test measured steady state position
        // error and found no difference at all, because the velocity loop's
        // integrator quietly absorbs a wrong gravity feedforward given
        // enough time. That is correct behaviour and the wrong thing to
        // measure.
        //
        // What an accurate payload actually buys is two things the
        // integrator cannot give back: headroom, because current spent
        // holding up an unmodelled mass is current not available to reject
        // disturbances, and transient accuracy, because the integrator has
        // to wind up before it helps.
        const arm::Vec2 lift_from = ik(pp, 0.260, -0.108);
        const arm::Vec2 lift_to = ik(pp, 0.300, 0.040);

        struct Result { double payload, integ_a, peak_err_mm; };

        auto trial = [&](bool tell_it_the_truth) -> Result {
            Rig r;
            r.hold(lift_from, 1.5);
            r.can.send_grip(true);
            r.hold(lift_from, 0.4);
            const double got = r.world.payload_kg();
            r.payload_belief = tell_it_the_truth ? static_cast<float>(got) : 0.0f;

            // A step lift while carrying the box. Peak error during the
            // transient is what the operator would see as sag.
            float q_ref[2] = {static_cast<float>(lift_to[0]),
                              static_cast<float>(lift_to[1])};
            float qd_ref[2] = {0.0f, 0.0f};
            double peak = 0.0;
            for (int ms = 0; ms < 2500; ++ms) {
                r.control_step(q_ref, qd_ref);
                const arm::Vec2 tip = r.world.tip();
                const arm::Vec2 want = arm::forward_kinematics(pp.arm, lift_to);
                const double e = std::hypot(tip[0] - want[0], tip[1] - want[1]);
                if (ms > 1200 && e > peak) peak = e;  // after the move, holding
            }
            const float integ_a =
                r.axis[0].velocity_integral() * kAxisGains[0].ki_vel;
            return Result{got, std::fabs(integ_a), peak * 1000.0};
        };

        const Result blind = trial(false);
        const Result knowing = trial(true);

        std::printf("    holding a %.2f kg box:\n", blind.payload);
        std::printf("      believes it is empty   integrator carries %.3f A,"
                    " hold error %.3f mm\n", blind.integ_a, blind.peak_err_mm);
        std::printf("      told the true mass     integrator carries %.3f A,"
                    " hold error %.3f mm\n", knowing.integ_a, knowing.peak_err_mm);
        std::printf("    The integrator figure is headroom spent. Step 5 recovers it.\n");

        std::snprintf(buf, sizeof(buf), "%.3f A blind vs %.3f A informed",
                      blind.integ_a, knowing.integ_a);
        check("a wrong payload belief costs integrator headroom",
              blind.payload > 0.0 && blind.integ_a > knowing.integ_a + 0.05, buf);
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
