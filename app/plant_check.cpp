// Step 1 check: is the machine physically right, and is the payload even
// observable?
//
// The last question is the one that matters. The entire project rests on the
// arm being able to work out what it is carrying from motor current alone. If
// gripping a half kilogram box does not move the current well clear of the
// noise, then no estimator can recover it and the design is wrong. Better to
// find that out now than after a day of filter work.
#include <cmath>
#include <cstdio>
#include <utility>

#include "sim/world.hpp"

using namespace ppa;

namespace {

constexpr double kDt = 1.0e-5;  // 10 us, set by the 1 ms winding constant
int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-44s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

// A crude joint space PD, enough to hold a pose so steady state currents can
// be measured. Not the real controller: that is step 4, and it will close
// current and velocity loops properly.
struct HoldPd {
    double kp{260.0};
    double kd{26.0};

    void apply(World& w, const arm::Vec2& q_ref) const {
        const arm::Vec2 q = w.q();
        const arm::Vec2 qd = w.qd();
        w.set_voltage(kp * (q_ref[0] - q[0]) - kd * qd[0],
                      kp * (q_ref[1] - q[1]) - kd * qd[1]);
    }
};

// Drive toward a pose for a while and return the mean current over the last
// fifth of the run, by which point the transient has gone.
struct Settled {
    double i1, i2, tip_x, tip_y;
};

Settled hold(World& w, const Faults& f, const arm::Vec2& q_ref, double seconds) {
    const HoldPd pd;
    const int steps = static_cast<int>(seconds / kDt);
    const int avg_from = steps - steps / 5;
    double s1 = 0.0, s2 = 0.0;
    int n = 0;
    for (int i = 0; i < steps; ++i) {
        pd.apply(w, q_ref);
        w.step(f, kDt);
        if (i >= avg_from) {
            s1 += w.current()[0];
            s2 += w.current()[1];
            ++n;
        }
    }
    const arm::Vec2 tip = w.tip();
    return Settled{s1 / n, s2 / n, tip[0], tip[1]};
}

arm::Vec2 ik_or_die(const arm::ArmParams& p, double x, double y, const char* what) {
    const arm::IkResult r =
        arm::inverse_kinematics(p, arm::Vec2{x, y}, arm::Elbow::kDown);
    if (!r.reachable) {
        std::printf("  target %s (%.3f, %.3f) is unreachable\n", what, x, y);
        ++g_failures;
        return arm::Vec2{0.0, 0.0};
    }
    return r.q;
}

}  // namespace

int main() {
    char buf[192];
    const arm::PlantParams pp = arm::default_params();
    Faults clean{};

    // ------------------------------------------------------------- geometry
    std::printf("geometry and kinematics\n");
    {
        World w(pp, 1u);
        const arm::ArmParams& a = w.bare_arm();
        std::snprintf(buf, sizeof(buf), "reach %.3f m to %.3f m",
                      std::fabs(a.l1 - a.l2), a.l1 + a.l2);
        check("workspace is an annulus", a.l1 > 0.0 && a.l2 > 0.0, buf);

        // Inverse then forward must land back on the target.
        const double tx = 0.260, ty = -0.110;
        const arm::Vec2 q = ik_or_die(a, tx, ty, "pick point");
        const arm::Vec2 tip = arm::forward_kinematics(a, q);
        const double err = std::hypot(tip[0] - tx, tip[1] - ty);
        std::snprintf(buf, sizeof(buf), "round trip error %.2e m", err);
        check("inverse then forward kinematics agree", err < 1e-9, buf);
    }

    // ------------------------------------------------------------ gravity
    std::printf("\ngravity\n");
    {
        World w(pp, 2u);
        const double y0 = w.tip()[1];
        for (int i = 0; i < 40000; ++i) {  // 0.4 s with no drive at all
            w.set_voltage(0.0, 0.0);
            w.step(clean, kDt);
        }
        const double y1 = w.tip()[1];
        std::snprintf(buf, sizeof(buf), "tip fell from %.4f to %.4f m", y0, y1);
        check("unpowered arm falls under gravity", y1 < y0 - 0.01, buf);
    }

    // ----------------------------------------------------- payload inertia
    std::printf("\npayload inertial parameters\n");
    {
        const arm::ArmParams bare = pp.arm;
        const arm::ArmParams with = arm::with_payload(bare, 0.5, bare.l2);

        // Hand calculation, from the formulas in payload.hpp:
        //   m'  = 0.8 + 0.5 = 1.3
        //   lc' = (0.8*0.100 + 0.5*0.200) / 1.3 = 0.13846
        //   I'  = 0.00267 + 0.8*(0.13846-0.100)^2 + 0.5*(0.200-0.13846)^2
        //       = 0.00267 + 0.001183 + 0.001893 = 0.005746
        std::snprintf(buf, sizeof(buf), "m2 %.4f kg, expected 1.3000", with.m2);
        check("payload mass adds", std::fabs(with.m2 - 1.3) < 1e-9, buf);
        std::snprintf(buf, sizeof(buf), "lc2 %.5f m, expected 0.13846", with.lc2);
        check("centre of mass shifts outward",
              std::fabs(with.lc2 - 0.138461538) < 1e-6, buf);
        std::snprintf(buf, sizeof(buf), "I2 %.6f, expected 0.005746", with.I2);
        check("inertia grows by parallel axis",
              std::fabs(with.I2 - 0.005746) < 5e-6, buf);
    }

    // ------------------------------------------------- the decisive check
    std::printf("\nis the payload observable from current alone?\n");
    {
        // Same pose, same controller, once empty and once holding a box.
        // Nothing else differs, so any current change is the payload.
        const arm::Vec2 q_ref = ik_or_die(pp.arm, 0.300, -0.020, "hold pose");

        World empty(pp, 3u);
        const Settled e = hold(empty, clean, q_ref, 1.5);

        World loaded(pp, 3u);
        // Place the box directly in the gripper by driving to it, gripping,
        // then returning to the same hold pose.
        const arm::Vec2 q_pick = ik_or_die(pp.arm, 0.260, -0.110, "box 2 top");
        hold(loaded, clean, q_pick, 1.2);
        loaded.command_grip(true);
        hold(loaded, clean, q_pick, 0.3);
        const bool picked = loaded.payload_kg() > 0.0;
        std::snprintf(buf, sizeof(buf), "payload %.3f kg, grip %.1f N",
                      loaded.payload_kg(), loaded.grip_force_n());
        check("gripper picked the box up", picked, buf);

        const Settled l = hold(loaded, clean, q_ref, 1.5);

        const double d1 = std::fabs(l.i1 - e.i1);
        const double d2 = std::fabs(l.i2 - e.i2);
        std::printf("    joint 1  empty %+.4f A   loaded %+.4f A   change %.4f A\n",
                    e.i1, l.i1, d1);
        std::printf("    joint 2  empty %+.4f A   loaded %+.4f A   change %.4f A\n",
                    e.i2, l.i2, d2);

        // A 12 bit current sense across a 6 A peak range resolves about
        // 3 mA, and a realistic sensor noise floor is a few tens of mA. A
        // change of 100 mA or more is comfortably measurable.
        std::snprintf(buf, sizeof(buf), "%.0f mA on joint 1, %.0f mA on joint 2",
                      d1 * 1000.0, d2 * 1000.0);
        check("payload moves current well clear of the noise floor",
              d1 > 0.10 || d2 > 0.10, buf);
    }

    // --------------------------------------------------------- contact
    std::printf("\ncontact with the table\n");
    {
        World w(pp, 4u);
        // Command a pose 15 mm below the table top. The arm cannot get
        // there, and the table must push back rather than be passed through.
        const arm::Vec2 q_ref = ik_or_die(pp.arm, 0.300, -0.165, "below table");
        hold(w, clean, q_ref, 1.0);
        const double tip_y = w.tip()[1];
        const double fy = w.contact_force()[1];
        std::snprintf(buf, sizeof(buf), "tip y %.4f m, table at %.4f m",
                      tip_y, w.layout().table_y);
        check("tool does not pass through the table",
              tip_y > w.layout().table_y - 0.004, buf);
        std::snprintf(buf, sizeof(buf), "normal force %.1f N, tau_ext %.3f / %.3f Nm",
                      fy, w.tau_ext()[0], w.tau_ext()[1]);
        check("contact produces a normal force and joint torque",
              fy > 1.0 && std::fabs(w.tau_ext()[0]) > 1e-3, buf);
    }

    // --------------------------------------------------------- dropping it
    std::printf("\nlosing the box\n");
    {
        World w(pp, 5u);
        const arm::Vec2 q_pick = ik_or_die(pp.arm, 0.260, -0.110, "box 2 top");
        hold(w, clean, q_pick, 1.2);
        w.command_grip(true);
        hold(w, clean, q_pick, 0.3);
        const arm::Vec2 q_up = ik_or_die(pp.arm, 0.260, -0.040, "lifted");
        hold(w, clean, q_up, 0.8);
        check("box lifted clear of the table", w.payload_kg() > 0.0, "");

        Faults leak{};
        leak.grip_leak = true;
        hold(w, leak, q_up, 1.0);
        std::snprintf(buf, sizeof(buf), "payload now %.3f kg, %u drop events",
                      w.payload_kg(), w.drop_events());
        check("grip leak drops the box",
              w.payload_kg() == 0.0 && w.drop_events() == 1, buf);
        std::snprintf(buf, sizeof(buf), "box 2 back at y %.4f, table %.4f",
                      w.box(1).y, w.layout().table_y + w.box(1).half_h);
        check("dropped box falls to the table",
              std::fabs(w.box(1).y - (w.layout().table_y + w.box(1).half_h)) < 1e-3, buf);
    }

    // --------------------------------------------------------- collision
    std::printf("\nhitting something unexpected\n");
    {
        World w(pp, 6u);
        Faults obs{};
        obs.obstacle_present = true;
        const arm::Vec2 q_ref = ik_or_die(pp.arm, w.layout().obstacle_x,
                                          w.layout().obstacle_y, "obstacle");
        hold(w, obs, q_ref, 1.0);
        std::snprintf(buf, sizeof(buf), "%u collisions, force %.1f N",
                      w.collision_events(),
                      std::hypot(w.contact_force()[0], w.contact_force()[1]));
        check("collision is felt as an external force",
              w.collision_events() > 0, buf);
    }

    // ---------------------- the hard problem, and how it gets solved
    std::printf("\nseparating a payload from a worn joint\n");
    {
        // Two things both make the motors pull more current: carrying a box,
        // and stiffer joints. A single current reading cannot tell them
        // apart. But they have different signatures.
        //
        // Gravity does not care which way the arm is moving. Coulomb
        // friction always opposes motion, so it flips sign when the arm
        // reverses. So sweep the same path out and back, and split the
        // current into the part that is the same in both directions and the
        // part that flips:
        //
        //   symmetric  = (i_out + i_return) / 2   ->  gravity and payload
        //   asymmetric = (i_out - i_return) / 2   ->  friction
        //
        // Same poses in both directions means the gravity term is identical
        // and cancels out of the asymmetric part exactly.
        //
        // This was found by accident: an earlier version of this test
        // averaged over the whole out and back sweep and measured a 6 mA
        // change from tripled friction, because the two halves cancelled.
        const arm::Vec2 qa = ik_or_die(pp.arm, 0.240, -0.020, "sweep start");
        const arm::Vec2 qb = ik_or_die(pp.arm, 0.340, -0.020, "sweep end");

        struct Split { double sym, asym, speed; };

        auto sweep = [&](const Faults& f, bool with_box) -> Split {
            World w(pp, 8u);
            if (with_box) {
                const arm::Vec2 qp = ik_or_die(pp.arm, 0.260, -0.110, "box 2");
                hold(w, f, qp, 1.2);
                w.command_grip(true);
                hold(w, f, qp, 0.3);
            }
            hold(w, f, qa, 0.8);

            const HoldPd pd;
            const int steps = static_cast<int>(1.6 / kDt);
            double out = 0.0, ret = 0.0, spd = 0.0;
            int n_out = 0, n_ret = 0;
            for (int i = 0; i < steps; ++i) {
                const double u = 0.5 - 0.5 * std::cos(2.0 * 3.14159265 *
                                 static_cast<double>(i) / steps);
                const arm::Vec2 qr{qa[0] + u * (qb[0] - qa[0]),
                                   qa[1] + u * (qb[1] - qa[1])};
                pd.apply(w, qr);
                w.step(f, kDt);
                spd += std::fabs(w.qd()[0]);
                if (i < steps / 2) { out += w.current()[0]; ++n_out; }
                else               { ret += w.current()[0]; ++n_ret; }
            }
            const double io = out / n_out, ir = ret / n_ret;
            return Split{0.5 * (io + ir), 0.5 * (io - ir), spd / steps};
        };

        Faults worn{};
        worn.friction_mult = 3.0f;

        const Split healthy = sweep(clean, false);
        const Split wornj   = sweep(worn, false);
        const Split loaded  = sweep(clean, true);

        std::printf("    mean joint 1 speed over the sweep: %.2f rad/s\n\n", healthy.speed);
        std::printf("    %-22s %12s %12s\n", "case", "symmetric", "asymmetric");
        std::printf("    %-22s %10.4f A %10.4f A\n", "healthy, empty", healthy.sym, healthy.asym);
        std::printf("    %-22s %10.4f A %10.4f A\n", "3x joint friction", wornj.sym, wornj.asym);
        std::printf("    %-22s %10.4f A %10.4f A\n", "healthy, 0.5 kg box", loaded.sym, loaded.asym);

        const double fric_on_asym = std::fabs(wornj.asym - healthy.asym);
        const double fric_on_sym  = std::fabs(wornj.sym - healthy.sym);
        const double load_on_asym = std::fabs(loaded.asym - healthy.asym);
        const double load_on_sym  = std::fabs(loaded.sym - healthy.sym);

        std::printf("\n    friction moved  symmetric by %5.0f mA, asymmetric by %5.0f mA\n",
                    fric_on_sym * 1000.0, fric_on_asym * 1000.0);
        std::printf("    payload moved   symmetric by %5.0f mA, asymmetric by %5.0f mA\n",
                    load_on_sym * 1000.0, load_on_asym * 1000.0);

        std::snprintf(buf, sizeof(buf), "%.0f mA asymmetric vs %.0f mA symmetric",
                      fric_on_asym * 1000.0, fric_on_sym * 1000.0);
        check("friction shows up mostly in the asymmetric part",
              fric_on_asym > 2.0 * fric_on_sym, buf);

        std::snprintf(buf, sizeof(buf), "%.0f mA symmetric vs %.0f mA asymmetric",
                      load_on_sym * 1000.0, load_on_asym * 1000.0);
        check("payload shows up mostly in the symmetric part",
              load_on_sym > 2.0 * load_on_asym, buf);
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
