// Fixed priority scheduler with measured timing.
//
// Five jobs at five different rates share one core. The gyro must be read a
// thousand times a second; the beacon sensor only speaks ten times a second.
// Something has to guarantee the fast job wins, and has to be able to prove
// afterwards that it did.
//
// Scheduling policy: fixed priority, run to completion, lowest number wins.
// A task that is released while a lower priority task is mid execution waits
// for it to finish rather than preempting it. That is the simpler half of
// what FreeRTOS does, and it is chosen deliberately: with no preemption there
// are no context switches, no per task stacks, and no shared state races, so
// the whole firmware fits in static storage and the worst case is analysable
// by hand. The cost is blocking time, which shows up in the jitter figures
// below rather than being hidden.
//
// How execution time is measured, which matters for reading the numbers:
//
//   release times run on the simulation clock, in virtual microseconds
//   execution cost is measured with the host's real clock, then scaled by
//   kHostToTargetScale to estimate what the same work would cost on the
//   target, and charged against the virtual clock
//
// So the deadline and jitter figures are a model, not a measurement of real
// hardware. The scale factor is the one honest knob: set it from a measured
// ratio once the same code has run on a real part. On target the host clock
// is replaced by the Cortex-M cycle counter and the scaling disappears.
#pragma once

#include "types.hpp"

namespace ppa {

using TaskFn = void (*)(void*);

struct TaskStats {
    u32 releases{0};          // times the task became due
    u32 runs{0};              // times the body actually executed
    u32 deadline_misses{0};   // finished later than its deadline allowed
    u32 max_jitter_us{0};     // worst gap between due and actually starting
    u32 min_exec_us{0xFFFFFFFFu};
    u32 max_exec_us{0};       // the worst case execution time, the WCET
    u64 sum_exec_us{0};
    u32 max_response_us{0};   // due until finished, what the deadline bounds

    u32 mean_exec_us() const {
        return runs ? static_cast<u32>(sum_exec_us / runs) : 0;
    }
};

class Scheduler {
public:
    static constexpr int kMaxTasks = 8;

    // How many nanoseconds of target time one nanosecond of host time
    // represents. The target is far SLOWER than the host, so this is well
    // above one.
    //
    // Estimated, not measured: an Apple silicon core retires a float sqrt
    // and add in roughly 1.5 ns with pipelining, while a Cortex-M4 at
    // 168 MHz needs about 18 cycles, or 107 ns, for the same work. That
    // ratio is about 70.
    //
    // Two caveats on any number read out of this scheduler. The ratio is
    // workload dependent, so integer heavy tasks scale differently from
    // float heavy ones. And the host clock resolves to a few tens of
    // nanoseconds, which at this scale is a couple of microseconds of noise
    // on every sample. Treat the figures as a budget model, and replace this
    // constant with a measured ratio once the same code has run on a part.
#if defined(__ARM_EABI__)
    // On target the cycle counter measures the target directly, so there is
    // nothing to scale. Every timing figure the scheduler reports on hardware
    // is measured rather than modelled.
    static constexpr float kTargetPerHostNs = 1.0f;
#else
    static constexpr float kTargetPerHostNs = 70.0f;
#endif

    // Lower priority number runs first.
    void add(const char* name, u32 period_us, u8 priority, u32 deadline_us,
             TaskFn fn, void* ctx);

    // Advance the scheduler to this virtual time, releasing and running
    // whatever is due. Called from the simulation loop after each plant step.
    void tick(TimeUs now_us);

    int task_count() const { return count_; }
    const char* name(int i) const { return t_[i].name; }
    u32 period_us(int i) const { return t_[i].period_us; }
    u8 priority(int i) const { return t_[i].priority; }
    const TaskStats& stats(int i) const { return t_[i].stats; }

    u32 total_deadline_misses() const;

    // Fraction of the virtual timeline the core spent executing tasks, which
    // is the number that says whether the design has headroom.
    float cpu_load() const;

private:
    struct Task {
        const char* name{nullptr};
        u32 period_us{0};
        u8 priority{0};
        u32 deadline_us{0};
        TaskFn fn{nullptr};
        void* ctx{nullptr};
        TimeUs next_release_us{0};
        TimeUs ideal_release_us{0};
        bool ready{false};
        TaskStats stats{};
    };

    Task t_[kMaxTasks]{};
    int count_{0};
    TimeUs cpu_free_at_us_{0};  // core is busy until this virtual time
    TimeUs busy_us_{0};
    TimeUs now_us_{0};
};

}  // namespace ppa
