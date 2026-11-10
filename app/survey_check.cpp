// Step 9 check: can the arm find its own work?
//
// Three boxes sit at x = 0.180, 0.260 and 0.340, each 50 mm wide with its top
// face 40 mm above the table. The arm is told none of that. It sweeps its
// laser across the table and has to come back with three objects in the right
// places, at the right height.
#include <cmath>
#include <cstdio>

#include "task/machine.hpp"

using namespace ppa;

namespace {

int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-48s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

// Run a survey to completion, returning the seconds it took.
double run_survey(Machine& m, double max_s = 30.0) {
    m.start_survey();
    const int n = static_cast<int>(max_s / 1.0e-3);
    for (int i = 0; i < n; ++i) {
        m.cycle();
        if (m.survey().finished()) return i * 1.0e-3;
    }
    return -1.0;
}

void report(const Machine& m) {
    const Survey& s = m.survey();
    std::printf("    %s, coverage %.0f%%, %d objects\n",
                Survey::state_name(s.state()), 100.0 * s.coverage(),
                s.object_count());
    std::printf("    %-8s %10s %10s %10s\n", "object", "centre x", "top y", "width");
    for (int i = 0; i < s.object_count(); ++i) {
        const DetectedObject& o = s.object(i);
        std::printf("    %-8d %10.4f %10.4f %10.4f\n", i + 1, o.centre_x,
                    o.top_y, o.width_m);
    }
}

}  // namespace

int main() {
    char buf[200];

    // The truth the arm is not told.
    const double truth_x[3] = {0.180, 0.260, 0.340};
    const double truth_top = -0.110;
    const double truth_w = 0.050;

    std::printf("sweeping a table the arm knows nothing about\n");
    {
        Machine m;
        for (int i = 0; i < 1200; ++i) m.cycle();  // settle and converge
        const double t = run_survey(m);
        report(m);
        std::printf("    took %.2f s\n", t);

        const Survey& s = m.survey();
        std::snprintf(buf, sizeof(buf), "%d found, 3 expected", s.object_count());
        check("finds three objects", s.object_count() == 3, buf);

        if (s.object_count() == 3) {
            double worst_x = 0.0, worst_top = 0.0, worst_w = 0.0;
            for (int i = 0; i < 3; ++i) {
                worst_x = std::fmax(worst_x,
                                    std::fabs(s.object(i).centre_x - truth_x[i]));
                worst_top = std::fmax(worst_top,
                                      std::fabs(s.object(i).top_y - truth_top));
                worst_w = std::fmax(worst_w,
                                    std::fabs(s.object(i).width_m - truth_w));
            }
            std::snprintf(buf, sizeof(buf), "worst centre error %.1f mm",
                          worst_x * 1000.0);
            check("centres within 6 mm of truth", worst_x < 0.006, buf);
            std::snprintf(buf, sizeof(buf), "worst height error %.1f mm",
                          worst_top * 1000.0);
            check("top faces within 5 mm of truth", worst_top < 0.005, buf);
            std::snprintf(buf, sizeof(buf), "worst width error %.1f mm",
                          worst_w * 1000.0);
            check("widths within 8 mm of truth", worst_w < 0.008, buf);
        }

        std::snprintf(buf, sizeof(buf), "%.0f%% of bins received a reading",
                      100.0 * s.coverage());
        check("the table was actually seen", s.coverage() > 0.9f, buf);
    }

    // --------------------------------------------- something extra is there
    std::printf("\nan obstacle that should not be on the table\n");
    {
        Machine m;
        m.faults().obstacle_present = true;
        for (int i = 0; i < 1200; ++i) m.cycle();
        run_survey(m);
        report(m);
        const Survey& s = m.survey();
        std::printf("    safety %s (%s)\n",
                    SafetySupervisor::state_name(m.safety().state()),
                    SafetySupervisor::reason_name(m.safety().reason()));
        std::snprintf(buf, sizeof(buf), "%d objects, 3 boxes plus the obstacle",
                      s.object_count());
        check("the extra object is seen", s.object_count() == 4, buf);
    }

    // ------------------------------------------------- a sensor that is out
    std::printf("\nsurveying with the ranger silent\n");
    {
        Machine m;
        for (int i = 0; i < 1200; ++i) m.cycle();
        m.faults().laser_silent = true;
        run_survey(m);
        const Survey& s = m.survey();
        std::printf("    %s, coverage %.0f%%, %d objects\n",
                    Survey::state_name(s.state()), 100.0 * s.coverage(),
                    s.object_count());
        // The important behaviour is that it refuses to report an empty table
        // as fact. No data is not the same as nothing there, and a sweep that
        // saw nothing must fail rather than hand back an empty list the
        // sequence would treat as the truth.
        std::snprintf(buf, sizeof(buf), "coverage %.0f%%, state %s",
                      100.0 * s.coverage(), Survey::state_name(s.state()));
        check("no data is reported as a failure, not an empty table",
              s.state() == SurveyState::kFailed && s.coverage() < 0.1f, buf);
    }

    // ------------------------------------------------ a noisy serial line
    std::printf("\nsurveying with a noisy ranger\n");
    {
        Machine m;
        for (int i = 0; i < 1200; ++i) m.cycle();
        m.faults().laser_dropouts = true;
        run_survey(m);
        report(m);
        const Survey& s = m.survey();
        std::snprintf(buf, sizeof(buf), "%d objects at %.0f%% coverage",
                      s.object_count(), 100.0 * s.coverage());
        check("dropped readings still leave the boxes findable",
              s.object_count() == 3, buf);
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
