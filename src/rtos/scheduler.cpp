#include "rtos/scheduler.hpp"

#if defined(__ARM_EABI__)
extern "C" unsigned long dwt_cycles(void);
extern "C" unsigned long cpu_hz(void);
#else
#include <chrono>
#endif

namespace ppa {

namespace {

// Where the execution time comes from, and why it differs by platform.
//
// On target this reads the Cortex-M cycle counter, which counts core clocks
// with no prescaler and no interrupt. That is a direct measurement of what a
// task cost, so no scaling is needed and kTargetPerHostNs is 1.
//
// On the host there is no such thing, so the host's own clock is read and
// scaled by an estimate. Those figures are a budget model and the header
// says so. The whole point of the target build is that they stop being one.
#if defined(__ARM_EABI__)
u64 host_now_ns() {
    return static_cast<u64>(dwt_cycles()) * 1000000000ull /
           static_cast<u64>(cpu_hz());
}
#else
u64 host_now_ns() {
    using clock = std::chrono::steady_clock;
    return static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            clock::now().time_since_epoch())
            .count());
}
#endif

}  // namespace

void Scheduler::add(const char* name, u32 period_us, u8 priority,
                    u32 deadline_us, TaskFn fn, void* ctx) {
    if (count_ >= kMaxTasks) return;
    Task& t = t_[count_++];
    t.name = name;
    t.period_us = period_us;
    t.priority = priority;
    // A deadline of zero means the implicit one: finish before the next
    // release. That is the usual assumption and it is what makes the task set
    // analysable, so it is the default rather than something to forget.
    t.deadline_us = deadline_us ? deadline_us : period_us;
    t.fn = fn;
    t.ctx = ctx;
    t.next_release_us = 0;
}

void Scheduler::tick(TimeUs now_us) {
    now_us_ = now_us;

    // Release everything that has come due. A task still marked ready from a
    // previous period has been released twice without running, which is an
    // overrun of the task set rather than of any single task, and it is
    // counted as a deadline miss at the moment it becomes unrecoverable.
    for (int i = 0; i < count_; ++i) {
        Task& t = t_[i];
        if (t.fn == nullptr) continue;
        while (now_us >= t.next_release_us) {
            if (t.ready) {
                ++t.stats.deadline_misses;
            } else {
                t.ready = true;
                t.ideal_release_us = t.next_release_us;
            }
            ++t.stats.releases;
            t.next_release_us += t.period_us;
        }
    }

    // Dispatch. Highest priority ready task first, run to completion, repeat
    // until the core is booked past the present moment or nothing is ready.
    for (;;) {
        if (cpu_free_at_us_ > now_us) return;

        int pick = -1;
        for (int i = 0; i < count_; ++i) {
            if (!t_[i].ready) continue;
            if (pick < 0 || t_[i].priority < t_[pick].priority) pick = i;
        }
        if (pick < 0) return;

        Task& t = t_[pick];
        t.ready = false;

        // The core cannot start before it is free, and cannot start before
        // the task was released.
        const TimeUs start = (cpu_free_at_us_ > t.ideal_release_us)
                                 ? cpu_free_at_us_
                                 : t.ideal_release_us;

        const u64 t0 = host_now_ns();
        t.fn(t.ctx);
        const u64 elapsed_ns = host_now_ns() - t0;

        // Charge at least one microsecond. A body that measures as zero on
        // the host still costs something real on the target, and letting it
        // be free would make the load figure a lie.
        u32 cost_us = static_cast<u32>(
            static_cast<float>(elapsed_ns) * kTargetPerHostNs / 1000.0f);
        if (cost_us == 0) cost_us = 1;

        const TimeUs finish = start + cost_us;
        cpu_free_at_us_ = finish;
        busy_us_ += cost_us;

        TaskStats& s = t.stats;
        ++s.runs;
        const u32 jitter = static_cast<u32>(start - t.ideal_release_us);
        if (jitter > s.max_jitter_us) s.max_jitter_us = jitter;
        if (cost_us < s.min_exec_us) s.min_exec_us = cost_us;
        if (cost_us > s.max_exec_us) s.max_exec_us = cost_us;
        s.sum_exec_us += cost_us;

        const u32 response = static_cast<u32>(finish - t.ideal_release_us);
        if (response > s.max_response_us) s.max_response_us = response;
        if (response > t.deadline_us) ++s.deadline_misses;
    }
}

u32 Scheduler::total_deadline_misses() const {
    u32 n = 0;
    for (int i = 0; i < count_; ++i) n += t_[i].stats.deadline_misses;
    return n;
}

float Scheduler::cpu_load() const {
    if (now_us_ == 0) return 0.0f;
    return static_cast<float>(busy_us_) / static_cast<float>(now_us_);
}

}  // namespace ppa
