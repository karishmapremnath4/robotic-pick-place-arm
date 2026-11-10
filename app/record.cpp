// Records a scripted run for the viewer.
//
// A fixed scenario, run unthrottled, writing everything the browser page
// needs to replay it: the arm, the boxes, the estimate, the forces, the
// events, and the configuration space map.
//
// The map is written from the real planner rather than recomputed in
// JavaScript. A second copy of the collision test in another language would
// drift from this one, and then the picture would be of a robot that does not
// exist.
#include <cmath>
#include <cstdio>

#include "task/machine.hpp"
#include "task/planner.hpp"

using namespace ppa;

namespace {

constexpr float kControlDt = 1.0e-3f;

std::FILE* g_csv = nullptr;
std::FILE* g_ev = nullptr;
double g_next_sample = 0.0;

void sample(const Machine& m) {
    const Telemetry& t = m.telemetry();
    if (t.t_s < g_next_sample) return;
    g_next_sample = t.t_s + 0.02;  // 50 Hz, enough for smooth playback

    std::fprintf(g_csv,
                 "%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%d,%d,%d",
                 t.t_s, t.true_q1, t.true_q2, t.est_q1, t.est_q2,
                 t.true_payload_kg, t.est_payload_kg, t.payload_sigma_kg,
                 t.true_force_n, t.est_force_n, t.laser_m,
                 static_cast<int>(t.task), static_cast<int>(t.safety),
                 static_cast<int>(t.surveying));
    for (int i = 0; i < t.n_boxes; ++i)
        std::fprintf(g_csv, ",%.4f,%.4f,%d", t.box_x[i], t.box_y[i],
                     t.box_held[i] ? 1 : 0);
    // Sensor health, so the viewer can light a lamp per sensor.
    for (int i = 0; i < kNumSensors; ++i)
        std::fprintf(g_csv, ",%d", static_cast<int>(t.health[i]));
    std::fprintf(g_csv, "\n");
}

void drain_events(Machine& m) {
    for (TaskEvent e = m.sequence().take_event(); e != TaskEvent::kNone;
         e = m.sequence().take_event()) {
        const Telemetry& t = m.telemetry();
        std::fprintf(g_ev, "%.3f\t%s\t%s\n", t.t_s, Sequence::event_name(e),
                     SafetySupervisor::state_name(t.safety));
        std::printf("  [%6.2fs] %s\n", t.t_s, Sequence::event_name(e));
    }
}

void run(Machine& m, double seconds) {
    const int n = static_cast<int>(seconds / kControlDt);
    for (int i = 0; i < n; ++i) {
        m.cycle();
        sample(m);
        drain_events(m);
    }
}

void run_until_survey_done(Machine& m, double max_s) {
    const int n = static_cast<int>(max_s / kControlDt);
    for (int i = 0; i < n; ++i) {
        m.cycle();
        sample(m);
        drain_events(m);
        if (m.survey().finished()) return;
    }
}

void run_job(Machine& m, const Job& j, double max_s) {
    m.start_job(j);
    const int n = static_cast<int>(max_s / kControlDt);
    for (int i = 0; i < n; ++i) {
        m.cycle();
        sample(m);
        drain_events(m);
        const TaskState s = m.sequence().state();
        if (s == TaskState::kDone || s == TaskState::kFailed) return;
    }
}

}  // namespace

int main() {
    Machine m;
    g_csv = std::fopen("data/replay.csv", "w");
    g_ev = std::fopen("data/replay_events.tsv", "w");
    if (!g_csv || !g_ev) {
        std::printf("could not open data/ for writing\n");
        return 1;
    }

    std::fprintf(g_csv,
                 "t,tq1,tq2,eq1,eq2,tpay,epay,psig,tforce,eforce,laser,"
                 "task,safety,survey");
    for (int i = 0; i < kMaxBoxes; ++i)
        std::fprintf(g_csv, ",b%dx,b%dy,b%dheld", i, i, i);
    for (int i = 0; i < kNumSensors; ++i) std::fprintf(g_csv, ",h%d", i);
    std::fprintf(g_csv, "\n");

    std::printf("recording\n");
    run(m, 1.0);

    std::printf("  survey\n");
    m.start_survey();
    run_until_survey_done(m, 20.0);
    run(m, 0.3);

    // The configuration space map, from what the survey actually saw.
    Scene scene{};
    scene.from_survey(m.survey(), static_cast<float>(m.world().layout().table_y));
    Planner planner;
    planner.build(scene);

    {
        std::FILE* f = std::fopen("data/scene.tsv", "w");
        std::fprintf(f, "table_y\t%.4f\n", scene.table_y);
        std::fprintf(f, "link1\t%.4f\n", kLink1M);
        std::fprintf(f, "link2\t%.4f\n", kLink2M);
        std::fprintf(f, "grid\t%d\n", kGridN);
        for (int i = 0; i < scene.n_boxes; ++i)
            std::fprintf(f, "object\t%.4f\t%.4f\t%.4f\n", scene.box_x[i],
                         scene.box_top_y[i], scene.box_half_w[i]);

        // One row of '.' and '#' per q1 bin. Compact, and readable in a
        // terminal if anyone wants to check it by eye.
        const float step = 6.28318530718f / kGridN;
        for (int i = 0; i < kGridN; ++i) {
            const float q1 = -3.14159265358979f + (i + 0.5f) * step;
            std::fprintf(f, "cspace\t");
            for (int j = 0; j < kGridN; ++j) {
                const float q2 = -3.14159265358979f + (j + 0.5f) * step;
                std::fprintf(f, "%c", planner.blocked_at(q1, q2) ? '#' : '.');
            }
            std::fprintf(f, "\n");
        }

        // A plan worth drawing: across the table at travel height.
        const float travel = scene.table_y + 0.110f;
        const IkSolution a = inverse_kinematics(0.340f, travel);
        const IkSolution b = inverse_kinematics(0.120f, travel);
        PlanResult r{};
        if (planner.plan(a.q1, a.q2, b.q1, b.q2, &r))
            for (int i = 0; i < r.n; ++i)
                std::fprintf(f, "path\t%.5f\t%.5f\n", r.q1[i], r.q2[i]);
        std::fclose(f);
        std::printf("  map: %d of %d poses blocked\n", planner.blocked_count(),
                    kGridCells);
    }

    // Two clean jobs, then one with a frozen encoder partway through.
    std::printf("  job 1\n");
    Job j{};
    j.pick_x = 0.260f;
    j.place_x = 0.120f;
    run_job(m, j, 25.0);
    run(m, 0.4);

    std::printf("  job 2\n");
    j.pick_x = 0.120f;
    j.place_x = 0.260f;
    run_job(m, j, 25.0);
    run(m, 0.4);

    std::printf("  job 3, with joint 1's encoder frozen partway\n");
    j.pick_x = 0.260f;
    j.place_x = 0.120f;
    m.start_job(j);
    bool injected = false;
    for (int i = 0; i < 25000; ++i) {
        m.cycle();
        sample(m);
        drain_events(m);
        if (!injected && m.sequence().state() == TaskState::kMoveAbovePlace) {
            m.faults().enc_stuck_joint = 1;
            injected = true;
            std::printf("  [%6.2fs] >> freezing joint 1 encoder\n",
                        m.telemetry().t_s);
        }
        const TaskState s = m.sequence().state();
        if (s == TaskState::kDone || s == TaskState::kFailed) break;
    }
    run(m, 1.0);

    std::fclose(g_csv);
    std::fclose(g_ev);
    std::printf("\n  %.1f s recorded to data/replay.csv\n", m.telemetry().t_s);
    std::printf("  events in data/replay_events.tsv, scene in data/scene.tsv\n");
    return 0;
}
