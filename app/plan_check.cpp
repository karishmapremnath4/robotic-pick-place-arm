// Step 10 check: does planning in configuration space actually avoid things,
// and was the straight line really unsafe?
//
// The second question matters as much as the first. Three bugs in this project
// were patched by interpolating a particular move, each time on the assumption
// that a straight line between two poses is fine. This is the test that makes
// that assumption measurable instead of assumed.
#include <chrono>
#include <cmath>
#include <cstdio>

#include "task/machine.hpp"
#include "task/planner.hpp"

using namespace ppa;

namespace {

int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-50s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

// Every waypoint of a plan must be a pose the arm may actually hold.
bool path_is_clear(const Planner& p, const PlanResult& r) {
    for (int i = 0; i < r.n; ++i)
        if (p.blocked_at(r.q1[i], r.q2[i])) return false;
    return true;
}

// And every step between waypoints, since the arm travels through them.
bool path_steps_clear(const Planner& p, const PlanResult& r) {
    for (int i = 1; i < r.n; ++i)
        if (!p.direct_clear(r.q1[i - 1], r.q2[i - 1], r.q1[i], r.q2[i]))
            return false;
    return true;
}

double tool_lowest_on_line(float a1, float a2, float b1, float b2) {
    double lowest = 1e9;
    const float d1 = wrap_pi(b1 - a1), d2 = wrap_pi(b2 - a2);
    for (int k = 0; k <= 200; ++k) {
        const float u = static_cast<float>(k) / 200.0f;
        const Vec<2> t = forward_kinematics(a1 + u * d1, a2 + u * d2);
        if (t[1] < lowest) lowest = t[1];
    }
    return lowest;
}

}  // namespace

int main() {
    char buf[220];
    const float table_y = -0.150f;

    // The scene the arm actually saw, from a real survey.
    Machine m;
    for (int i = 0; i < 1200; ++i) m.cycle();
    m.start_survey();
    for (int i = 0; i < 30000 && !m.survey().finished(); ++i) m.cycle();

    Scene scene{};
    scene.from_survey(m.survey(), table_y);
    std::printf("scene from the survey: %d boxes\n", scene.n_boxes);
    for (int i = 0; i < scene.n_boxes; ++i)
        std::printf("    box %d at x %.3f, top %.3f, half width %.3f\n", i + 1,
                    scene.box_x[i], scene.box_top_y[i], scene.box_half_w[i]);

    Planner planner;
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    planner.build(scene);
    const double build_ms =
        std::chrono::duration<double, std::milli>(clock::now() - t0).count();

    std::printf("\nthe map of forbidden poses\n");
    std::printf("    %d of %d cells blocked (%.0f%%), built in %.1f ms on this host\n",
                planner.blocked_count(), kGridCells,
                100.0 * planner.blocked_count() / kGridCells, build_ms);
    std::snprintf(buf, sizeof(buf), "%d blocked", planner.blocked_count());
    check("some poses are forbidden and some are not",
          planner.blocked_count() > 0 && planner.blocked_count() < kGridCells, buf);

    // A pose that drives the tool through the table must be forbidden.
    {
        const IkSolution under = inverse_kinematics(0.250f, table_y - 0.040f);
        const bool blocked = under.reachable &&
                             planner.blocked_at(under.q1, under.q2);
        check("a pose below the table is forbidden", blocked, "");

        const IkSolution safe = inverse_kinematics(0.250f, table_y + 0.110f);
        check("a pose well clear of everything is allowed",
              safe.reachable && !planner.blocked_at(safe.q1, safe.q2), "");
    }

    // ----------------------------- was the straight line really unsafe?
    std::printf("\nthe assumption three fixes were built on\n");
    {
        const float travel = table_y + 0.110f;
        const IkSolution a = inverse_kinematics(0.260f, travel);
        const IkSolution b = inverse_kinematics(0.120f, travel);

        const double lowest = tool_lowest_on_line(a.q1, a.q2, b.q1, b.q2);
        std::printf("    travelling from x 0.260 to x 0.120, both at y %.3f:\n", travel);
        std::printf("    the straight joint space line sags the tool to"
                    " y = %.4f, %.0f mm\n", lowest, 1000.0 * (travel - lowest));
        std::printf("    the table is at y = %.4f, which is %.0f mm lower still\n",
                    table_y, 1000.0 * (lowest - table_y));

        // This is a correction, not a feature. Step 7 claimed this move
        // swung the tool into the table and produced 2628 N, and the
        // measurement says it sags 14 mm and stays 96 mm clear. The force
        // came from the held box geometry bug found later in the same step.
        std::snprintf(buf, sizeof(buf), "sags %.0f mm, stays %.0f mm clear",
                      1000.0 * (travel - lowest), 1000.0 * (lowest - table_y));
        check("the straight line sags, but nowhere near the table",
              lowest < travel && lowest > table_y, buf);

        std::snprintf(buf, sizeof(buf), "direct line clear: %s",
                      planner.direct_clear(a.q1, a.q2, b.q1, b.q2) ? "yes" : "no");
        check("so for the bare table the straight line is in fact fine",
              planner.direct_clear(a.q1, a.q2, b.q1, b.q2), buf);

        // And a plan still works, which is what matters: planning must not
        // make the easy case worse.
        PlanResult r{};
        const bool ok = planner.plan(a.q1, a.q2, b.q1, b.q2, &r);
        std::printf("    planned route: %d waypoints, %d cells searched\n", r.n,
                    r.cells_searched);
        check("and a plan for it is found anyway", ok && r.n >= 2, "");
        check("every waypoint is a pose the arm may hold",
              path_is_clear(planner, r), "");
        check("and every step between them is clear too",
              path_steps_clear(planner, r), "");
    }

    // ------------------------------------------------ around an obstacle
    std::printf("\nrouting around an obstacle the arm surveyed\n");
    {
        Machine m2;
        m2.faults().obstacle_present = true;
        for (int i = 0; i < 1200; ++i) m2.cycle();
        m2.start_survey();
        for (int i = 0; i < 30000 && !m2.survey().finished(); ++i) m2.cycle();

        Scene s2{};
        s2.from_survey(m2.survey(), table_y);
        std::printf("    survey found %d objects\n", s2.n_boxes);

        Planner p2;
        p2.build(s2);
        std::printf("    %d of %d cells blocked (%.0f%%)\n", p2.blocked_count(),
                    kGridCells, 100.0 * p2.blocked_count() / kGridCells);
        std::snprintf(buf, sizeof(buf), "%d blocked with the obstacle vs %d without",
                      p2.blocked_count(), planner.blocked_count());
        check("the extra object forbids more poses",
              p2.blocked_count() > planner.blocked_count(), buf);

        const IkSolution a = inverse_kinematics(0.150f, table_y + 0.110f);
        const IkSolution b = inverse_kinematics(0.380f, table_y + 0.110f);

        // THIS is the case that justifies planning. The obstacle's top
        // stands into the travel plane, so a straight line at travel height
        // goes through it, and no amount of interpolating in Cartesian space
        // helps: a Cartesian straight line goes through it too.
        const bool direct = p2.direct_clear(a.q1, a.q2, b.q1, b.q2);
        std::snprintf(buf, sizeof(buf), "obstacle top at %.3f, travel plane at %.3f",
                      s2.box_top_y[2], table_y + 0.110f);
        check("the straight line is NOT clear here", !direct, buf);

        PlanResult r{};
        const bool ok = p2.plan(a.q1, a.q2, b.q1, b.q2, &r);
        std::snprintf(buf, sizeof(buf), "%d waypoints", r.n);
        check("but a route past the obstacle exists", ok, buf);
        check("and it never enters a forbidden pose",
              path_is_clear(p2, r) && path_steps_clear(p2, r), "");
    }

    // ------------------------------------------- when there is no answer
    std::printf("\nwhen there is no route\n");
    {
        // The goal is inside a box. There is nowhere to arrive, and the
        // planner has to say that rather than return a path into it.
        const IkSolution inside =
            inverse_kinematics(scene.box_x[1], scene.box_top_y[1] - 0.015f);
        PlanResult r{};
        const bool ok = planner.plan(0.9f, -1.6f, inside.q1, inside.q2, &r);
        std::snprintf(buf, sizeof(buf), "found=%d goal_blocked=%d",
                      ok ? 1 : 0, r.goal_blocked ? 1 : 0);
        check("a goal inside an object is refused, and says why",
              !ok && r.goal_blocked, buf);
    }

    // ---------------------------------------------------------- cost
    std::printf("\nwhat it costs\n");
    {
        const IkSolution a = inverse_kinematics(0.260f, table_y + 0.110f);
        const IkSolution b = inverse_kinematics(0.120f, table_y + 0.110f);
        PlanResult r{};
        auto t1 = clock::now();
        for (int i = 0; i < 20; ++i) planner.plan(a.q1, a.q2, b.q1, b.q2, &r);
        const double plan_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t1).count() / 20.0;

        std::printf("    building the map   %.1f ms\n", build_ms);
        std::printf("    one plan           %.2f ms\n", plan_ms);
        std::printf("    memory             %d bytes blocked bitmap, %d bytes"
                    " distance field\n",
                    static_cast<int>((kGridCells + 31) / 32 * 4),
                    static_cast<int>(kGridCells * 2));
        std::printf("    These are host figures. A Cortex-M4 is roughly seventy\n");
        std::printf("    times slower per float operation, so expect the map to\n");
        std::printf("    cost on the order of a second there. It is built once\n");
        std::printf("    per survey, not once per move, which is why that is\n");
        std::printf("    affordable and why the two were separated.\n");

        check("planning is fast enough to do between moves", plan_ms < 20.0, "");
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
