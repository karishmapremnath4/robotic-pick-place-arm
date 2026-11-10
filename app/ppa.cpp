// PPA-100 interactive console.
//
// Runs the machine in real time and takes typed commands while it runs. This
// is the application the whole project exists to make possible: you tell the
// arm to do a job, it does it by itself, you break a sensor while it works,
// and you watch what it does about it.
//
// Input is polled rather than read, because the control loop cannot stop to
// wait for a keystroke. A blocking read here would freeze the arm mid move,
// which is exactly the behaviour the rest of this project is built to avoid.
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "task/machine.hpp"
#include "task/planner.hpp"

using namespace ppa;

namespace {

constexpr float kControlDt = 1.0e-3f;

struct Console {
    Machine m{};
    double speed{1.0};        // multiples of real time, 0 means unthrottled
    bool running{true};
    bool auto_mode{false};
    int auto_phase{0};
    bool telemetry_on{true};
    bool report_survey{false};
    bool surveyed{false};
    Planner planner{};
    SafetyState last_safety{SafetyState::kNormal};
    std::FILE* csv{nullptr};
    std::FILE* events{nullptr};
    double next_status_s{0.0};
    double next_csv_s{0.0};

    ~Console() {
        if (csv) std::fclose(csv);
        if (events) std::fclose(events);
    }
};

void print_help() {
    std::printf(
        "\n"
        "  survey                              sweep the table and find the boxes\n"
        "  plan <x>                            route to x at travel height\n"
        "  job <pick_x> <place_x> [expect_kg]   run one pick and place\n"
        "  auto on | auto off                  shuttle a box back and forth\n"
        "  status                              full state dump\n"
        "  ack                                 clear a latched safe stop\n"
        "  speed <n> | speed max               1 is real time\n"
        "  clear                               clear every injected fault\n"
        "  quit\n"
        "\n"
        "  inject imu stuck            gyro freezes, still answers\n"
        "  inject imu nack             gyro stops acknowledging\n"
        "  inject imu noise <mult>     degraded part\n"
        "  inject enc stuck <1|2>      one joint encoder freezes\n"
        "  inject enc frames           encoder raises its error flag\n"
        "  inject can busoff           the whole bus detaches\n"
        "  inject can silent <node>    1,2 drives  3 gripper  4 safety relay\n"
        "  inject current bias <j> <a> a drifting current sensor\n"
        "  inject friction <mult>      worn joints\n"
        "  inject grip leak            the clamp bleeds away\n"
        "  inject obstacle             something in the path\n"
        "  inject laser dropouts       a noisy serial line\n"
        "  inject laser silent         the ranger stops talking\n"
        "\n");
}

void print_status(Console& c) {
    const Telemetry& t = c.m.telemetry();
    std::printf("\n  t = %.2f s     task %-18s safety %s",
                t.t_s, Sequence::state_name(t.task),
                SafetySupervisor::state_name(t.safety));
    if (t.trip != TripReason::kNone)
        std::printf("  (%s)", SafetySupervisor::reason_name(t.trip));
    std::printf("\n");
    std::printf("  tool      true (%+.4f, %+.4f) m      est (%+.4f, %+.4f) m\n",
                t.true_x, t.true_y, t.est_x, t.est_y);
    std::printf("  payload   true %.3f kg               est %.3f +- %.3f kg\n",
                t.true_payload_kg, t.est_payload_kg, t.payload_sigma_kg);
    std::printf("  contact   true %7.2f N               est %7.2f N\n",
                t.true_force_n, t.est_force_n);
    std::printf("  current   %+.3f A, %+.3f A     clamp %5.1f N   friction x%.2f\n",
                t.current1_a, t.current2_a, t.clamp_n, t.friction_scale);
    std::printf("  laser     %.3f m\n", t.laser_m);
    std::printf("  sensors   ");
    for (int i = 0; i < kNumSensors; ++i)
        std::printf("%s:%s  ", SensorHealth::name(static_cast<SensorId>(i)),
                    SensorHealth::health_name(t.health[i]));
    std::printf("\n  jobs      %u done, %u failed\n\n",
                c.m.sequence().jobs_completed(), c.m.sequence().jobs_failed());
}

void status_line(const Telemetry& t) {
    std::printf("[%7.2fs] %-18s %-9s  tool (%+.3f,%+.3f)  err %5.1f mm  "
                "load %.3f kg  force %6.1f N\n",
                t.t_s, Sequence::state_name(t.task),
                SafetySupervisor::state_name(t.safety), t.true_x, t.true_y,
                1000.0f * std::sqrt((t.true_x - t.est_x) * (t.true_x - t.est_x) +
                                    (t.true_y - t.est_y) * (t.true_y - t.est_y)),
                t.est_payload_kg, t.true_force_n);
    std::fflush(stdout);
}

bool eq(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

void do_inject(Console& c, int n, char** w) {
    Faults& f = c.m.faults();
    if (n < 2) { std::printf("  inject what? try 'help'\n"); return; }

    if (eq(w[1], "imu")) {
        if (n > 2 && eq(w[2], "stuck")) { f.imu_stuck = true; std::printf("  gyro frozen\n"); return; }
        if (n > 2 && eq(w[2], "nack")) { f.imu_nack = true; std::printf("  gyro not acknowledging\n"); return; }
        if (n > 3 && eq(w[2], "noise")) {
            f.imu_noise_mult = static_cast<float>(std::atof(w[3]));
            std::printf("  gyro noise x%.1f\n", f.imu_noise_mult);
            return;
        }
    } else if (eq(w[1], "enc")) {
        if (n > 3 && eq(w[2], "stuck")) {
            f.enc_stuck_joint = std::atoi(w[3]);
            std::printf("  joint %d encoder frozen\n", f.enc_stuck_joint);
            return;
        }
        if (n > 2 && eq(w[2], "frames")) { f.enc_error_frames = true; std::printf("  encoder error flag set\n"); return; }
    } else if (eq(w[1], "can")) {
        if (n > 2 && eq(w[2], "busoff")) { f.can_bus_off = true; std::printf("  CAN bus off\n"); return; }
        if (n > 3 && eq(w[2], "silent")) {
            f.can_silent_node = static_cast<u8>(std::atoi(w[3]));
            std::printf("  node %u silent\n", f.can_silent_node);
            return;
        }
    } else if (eq(w[1], "current") && n > 4 && eq(w[2], "bias")) {
        const int j = std::atoi(w[3]) - 1;
        if (j == 0 || j == 1) {
            f.current_bias_a[j] = static_cast<float>(std::atof(w[4]));
            std::printf("  joint %d current sensor biased %+.3f A\n", j + 1,
                        f.current_bias_a[j]);
        }
        return;
    } else if (eq(w[1], "friction") && n > 2) {
        f.friction_mult = static_cast<float>(std::atof(w[2]));
        std::printf("  joint friction x%.1f\n", f.friction_mult);
        return;
    } else if (eq(w[1], "grip") && n > 2 && eq(w[2], "leak")) {
        f.grip_leak = true; std::printf("  clamp force bleeding away\n"); return;
    } else if (eq(w[1], "obstacle")) {
        f.obstacle_present = true; std::printf("  obstacle placed in the path\n"); return;
    } else if (eq(w[1], "laser")) {
        if (n > 2 && eq(w[2], "dropouts")) { f.laser_dropouts = true; std::printf("  serial line noisy\n"); return; }
        if (n > 2 && eq(w[2], "silent")) { f.laser_silent = true; std::printf("  ranger silent\n"); return; }
    }
    std::printf("  did not understand that injection, try 'help'\n");
}

void handle(Console& c, char* line) {
    char* w[8];
    int n = 0;
    for (char* t = std::strtok(line, " \t\n"); t && n < 8;
         t = std::strtok(nullptr, " \t\n"))
        w[n++] = t;
    if (n == 0) return;

    if (eq(w[0], "help") || eq(w[0], "?")) { print_help(); return; }
    if (eq(w[0], "quit") || eq(w[0], "q")) { c.running = false; return; }
    if (eq(w[0], "status") || eq(w[0], "s")) { print_status(c); return; }
    if (eq(w[0], "ack")) { c.m.acknowledge(); std::printf("  safe stop acknowledged\n"); return; }
    if (eq(w[0], "clear")) { c.m.faults().clear(); std::printf("  all faults cleared\n"); return; }

    if (eq(w[0], "speed") && n > 1) {
        c.speed = eq(w[1], "max") ? 0.0 : std::atof(w[1]);
        std::printf("  speed %s\n", c.speed == 0.0 ? "unthrottled" : w[1]);
        return;
    }
    if (eq(w[0], "job") && n >= 3) {
        Job j{};
        j.pick_x = static_cast<float>(std::atof(w[1]));
        j.place_x = static_cast<float>(std::atof(w[2]));
        if (n > 3) j.expect_kg = static_cast<float>(std::atof(w[3]));
        c.auto_mode = false;
        c.m.start_job(j);
        std::printf("  job: pick at x %.3f, place at x %.3f", j.pick_x, j.place_x);
        if (j.expect_kg > 0.0f) std::printf(", expecting %.3f kg", j.expect_kg);
        std::printf("\n");
        return;
    }
    if (eq(w[0], "auto") && n > 1) {
        c.auto_mode = eq(w[1], "on");
        std::printf("  auto %s\n", c.auto_mode ? "on, shuttling a box" : "off");
        return;
    }
    if (eq(w[0], "plan") && n > 1) {
        if (!c.surveyed) {
            std::printf("  survey the table first, so the plan knows what to"
                        " avoid\n");
            return;
        }
        Scene sc{};
        sc.from_survey(c.m.survey(),
                       static_cast<float>(c.m.world().layout().table_y));
        c.planner.build(sc);

        const Telemetry& t = c.m.telemetry();
        const float travel = sc.table_y + 0.110f;
        const IkSolution to =
            inverse_kinematics(static_cast<float>(std::atof(w[1])), travel);
        if (!to.reachable) { std::printf("  that target is out of reach\n"); return; }

        PlanResult r{};
        const bool ok = c.planner.plan(t.est_q1, t.est_q2, to.q1, to.q2, &r);
        std::printf("  %d of %d poses forbidden by what the survey saw\n",
                    c.planner.blocked_count(), kGridCells);
        if (!ok) {
            std::printf("  no route%s%s\n",
                        r.goal_blocked ? ", the target pose is inside something" : "",
                        r.start_blocked ? ", and the arm is already in one" : "");
            return;
        }
        std::printf("  route found: %d waypoints, %d poses examined\n", r.n,
                    r.cells_searched);
        std::printf("  straight line to it is %s\n",
                    c.planner.direct_clear(t.est_q1, t.est_q2, to.q1, to.q2)
                        ? "already clear" : "NOT clear, so the detour matters");
        for (int i = 0; i < r.n && i < 8; ++i) {
            const Vec<2> p = forward_kinematics(r.q1[i], r.q2[i]);
            std::printf("    %2d  q (%+.3f, %+.3f)  tool (%+.3f, %+.3f)\n", i + 1,
                        r.q1[i], r.q2[i], p[0], p[1]);
        }
        if (r.n > 8) std::printf("    ... %d more\n", r.n - 8);
        return;
    }
    if (eq(w[0], "survey")) {
        c.auto_mode = false;
        c.m.start_survey();
        c.report_survey = true;
        std::printf("  sweeping the table\n");
        return;
    }
    if (eq(w[0], "inject")) { do_inject(c, n, w); return; }

    std::printf("  unknown command '%s', try 'help'\n", w[0]);
}

// Keep the next job coming while auto mode is on.
void service_auto(Console& c) {
    if (!c.auto_mode) return;
    if (c.m.surveying()) return;

    // Never start a job while the arm is stopped. A latched safe stop waits
    // for a person, and the first version happily restarted the job every
    // cycle instead: a hundred job starts and a hundred aborts per second,
    // which is both useless and the fastest way to hide the original fault.
    if (!c.m.safety().motion_allowed()) return;

    const TaskState s = c.m.sequence().state();
    if (s != TaskState::kIdle && s != TaskState::kDone && s != TaskState::kFailed)
        return;

    // Only advance the shuttle when the last job actually succeeded. The
    // first version flipped on every attempt, so after one failure it went
    // looking for a box at the place position it had never reached, failed
    // again, and alternated between two empty spots indefinitely.
    if (s == TaskState::kDone) c.auto_phase ^= 1;

    Job j{};
    if (c.auto_phase == 0) { j.pick_x = 0.260f; j.place_x = 0.120f; }
    else { j.pick_x = 0.120f; j.place_x = 0.260f; }
    c.m.start_job(j);
}

void open_logs(Console& c) {
    c.csv = std::fopen("data/run.csv", "w");
    if (c.csv)
        std::fprintf(c.csv,
                     "t_s,true_x,true_y,est_x,est_y,true_payload,est_payload,"
                     "payload_sigma,true_force,est_force,i1,i2,clamp,laser,"
                     "friction,task,safety\n");
    c.events = std::fopen("data/events.jsonl", "w");
}

}  // namespace

int main(int argc, char** argv) {
    Console c;
    open_logs(c);

    // A seeded run is reproducible, so a session that found something
    // interesting can be replayed exactly.
    std::printf("\nPPA-100 pick and place arm\n");
    std::printf("  table at y = %.3f m, boxes at x = 0.180, 0.260, 0.340\n",
                c.m.world().layout().table_y);
    std::printf("  type 'help' for commands, 'auto on' to just watch it work\n\n");

    // Non interactive use: run a fixed script and exit. Useful for recording
    // a run without a person at the keyboard.
    if (argc > 1 && eq(argv[1], "--demo")) {
        c.auto_mode = true;
        c.speed = 0.0;
    }

    using clock = std::chrono::steady_clock;
    auto wall_start = clock::now();
    double sim_elapsed = 0.0;

    while (c.running) {
        // ---- typed input, without ever blocking the loop ------------------
        struct pollfd pfd{};
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            char line[256];
            if (std::fgets(line, sizeof(line), stdin)) {
                handle(c, line);
            } else {
                c.running = false;  // stdin closed
            }
        }

        service_auto(c);
        c.m.cycle();

        // A survey that has just finished reports what it found.
        if (c.report_survey && c.m.survey().finished()) {
            c.report_survey = false;
            const Survey& sv = c.m.survey();
            std::printf("\n  %s, %.0f%% of the table seen, %d objects\n",
                        Survey::state_name(sv.state()), 100.0 * sv.coverage(),
                        sv.object_count());
            for (int i = 0; i < sv.object_count(); ++i) {
                const DetectedObject& o = sv.object(i);
                std::printf("    object %d at x %.3f, top %.3f m, %.0f mm wide\n",
                            i + 1, o.centre_x, o.top_y, o.width_m * 1000.0);
            }
            std::printf("\n");
            c.surveyed = (sv.state() == SurveyState::kDone);
        }
        sim_elapsed += kControlDt;

        // ---- events ------------------------------------------------------
        for (TaskEvent e = c.m.sequence().take_event(); e != TaskEvent::kNone;
             e = c.m.sequence().take_event()) {
            const Telemetry& t = c.m.telemetry();
            std::printf("  >> [%7.2fs] %s", t.t_s, Sequence::event_name(e));
            if (e == TaskEvent::kAbortedBySafety) {
                std::printf("  (%s)", SafetySupervisor::reason_name(t.trip));
                std::printf("\n     payload %.3f +- %.3f kg, force %.1f N",
                            t.est_payload_kg, t.payload_sigma_kg, t.est_force_n);
            }
            std::printf("\n");
            std::fflush(stdout);
            if (c.events) {
                std::fprintf(c.events,
                             "{\"t\":%.3f,\"event\":\"%s\",\"task\":\"%s\","
                             "\"safety\":\"%s\",\"payload\":%.3f}\n",
                             t.t_s, Sequence::event_name(e),
                             Sequence::state_name(t.task),
                             SafetySupervisor::state_name(t.safety),
                             t.est_payload_kg);
                std::fflush(c.events);
            }
        }

        const Telemetry& t = c.m.telemetry();

        // ---- safety transitions, logged as events -------------------------
        // The thing an operator most needs to be told is that the arm has
        // stopped, and that is not a task event: safety sits above the task
        // layer and can fire when no job is running at all.
        if (t.safety != c.last_safety) {
            c.last_safety = t.safety;
            std::printf("  >> [%7.2fs] safety %s (%s)\n", t.t_s,
                        SafetySupervisor::state_name(t.safety),
                        SafetySupervisor::reason_name(t.trip));
            std::fflush(stdout);
            if (c.events) {
                std::fprintf(c.events,
                             "{\"t\":%.3f,\"event\":\"safety %s\","
                             "\"task\":\"%s\",\"safety\":\"%s\","
                             "\"reason\":\"%s\",\"payload\":%.3f}\n",
                             t.t_s, SafetySupervisor::state_name(t.safety),
                             Sequence::state_name(t.task),
                             SafetySupervisor::state_name(t.safety),
                             SafetySupervisor::reason_name(t.trip),
                             t.est_payload_kg);
                std::fflush(c.events);
            }
        }

        // ---- periodic output ---------------------------------------------
        if (t.t_s >= c.next_status_s) {
            c.next_status_s = t.t_s + 0.5;
            if (c.telemetry_on) status_line(t);
        }
        if (c.csv && t.t_s >= c.next_csv_s) {
            c.next_csv_s = t.t_s + 0.02;
            std::fprintf(c.csv,
                         "%.4f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,"
                         "%.4f,%.4f,%.2f,%.4f,%.3f,%d,%d\n",
                         t.t_s, t.true_x, t.true_y, t.est_x, t.est_y,
                         t.true_payload_kg, t.est_payload_kg, t.payload_sigma_kg,
                         t.true_force_n, t.est_force_n, t.current1_a,
                         t.current2_a, t.clamp_n, t.laser_m, t.friction_scale,
                         static_cast<int>(t.task), static_cast<int>(t.safety));
        }

        // ---- pace to the wall clock --------------------------------------
        // Without this the arm would run sixty times faster than reality and
        // be impossible to watch, let alone type at.
        if (c.speed > 0.0) {
            const double want = sim_elapsed / c.speed;
            const double have =
                std::chrono::duration<double>(clock::now() - wall_start).count();
            if (want > have) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(want - have));
            } else if (have - want > 0.5) {
                // Fell badly behind, which on a laptop usually means the
                // process was descheduled. Resync rather than sprinting to
                // catch up, because sprinting makes the display jump.
                wall_start = clock::now();
                sim_elapsed = 0.0;
            }
        }

        if (argc > 1 && eq(argv[1], "--demo") && t.t_s > 40.0) c.running = false;
    }

    std::printf("\n");
    print_status(c);
    std::printf("  telemetry in data/run.csv, events in data/events.jsonl\n\n");
    return 0;
}
