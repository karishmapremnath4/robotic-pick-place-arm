// Fixed capacity ring buffer.
//
// Used for the CAN receive queue and for the inter task queues. Capacity is a
// template parameter so the storage is inline and nothing allocates. One
// producer and one consumer only: the head is written by the producer and the
// tail by the consumer, so on a single core microcontroller no lock is needed
// for that pairing.
//
// Overflow drops the newest item and counts it rather than overwriting the
// oldest. For a CAN receive queue a dropped frame is honest data loss that the
// firmware should know about, and silently losing the oldest frame would hide
// it.
#pragma once

#include "types.hpp"

namespace ppa {

template <typename T, int N>
class Ring {
public:
    static constexpr int kCapacity = N;

    bool push(const T& item) {
        const u32 next = (head_ + 1) % N;
        if (next == tail_) {
            ++dropped_;
            return false;
        }
        buf_[head_] = item;
        head_ = next;
        if (size() > high_water_) high_water_ = size();
        return true;
    }

    bool pop(T* out) {
        if (tail_ == head_) return false;
        *out = buf_[tail_];
        tail_ = (tail_ + 1) % N;
        return true;
    }

    bool empty() const { return head_ == tail_; }
    u32 size() const { return (head_ + N - tail_) % N; }

    // Both of these are diagnostics. The high water mark is what tells you
    // whether the capacity was chosen sensibly, and it is the number to quote
    // when justifying a queue depth.
    u32 dropped() const { return dropped_; }
    u32 high_water() const { return high_water_; }

    void clear() {
        head_ = tail_ = 0;
    }

private:
    T buf_[N]{};
    u32 head_{0};
    u32 tail_{0};
    u32 dropped_{0};
    u32 high_water_{0};
};

}  // namespace ppa
