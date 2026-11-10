// Deterministic noise source.
//
// The simulation is only useful as a test bench if a run can be repeated
// exactly, so this is a seeded xorshift rather than anything from <random>.
// Same seed, same command sequence, same numbers, on any platform. That is
// what lets a fault case be reduced to a regression test.
#pragma once

#include <cmath>

#include "types.hpp"

namespace ppa {

class Rng {
public:
    explicit Rng(u64 seed) : s_(seed ? seed : 0x9E3779B97F4A7C15ull) {}

    // xorshift64star. Cheap, no state beyond one word, good enough for
    // sensor noise and nowhere near good enough for cryptography.
    u64 next_u64() {
        s_ ^= s_ >> 12;
        s_ ^= s_ << 25;
        s_ ^= s_ >> 27;
        return s_ * 0x2545F4914F6CDD1Dull;
    }

    // Uniform on [0, 1).
    float uniform() {
        return static_cast<float>(next_u64() >> 40) * (1.0f / 16777216.0f);
    }

    // Zero mean unit variance, Box Muller. Both values of each pair are used
    // so the cost per sample is half a sin and half a log.
    float normal() {
        if (have_spare_) {
            have_spare_ = false;
            return spare_;
        }
        float u1 = uniform();
        if (u1 < 1e-7f) u1 = 1e-7f;
        const float u2 = uniform();
        const float mag = std::sqrt(-2.0f * std::log(u1));
        constexpr float kTwoPi = 6.28318530718f;
        spare_ = mag * std::sin(kTwoPi * u2);
        have_spare_ = true;
        return mag * std::cos(kTwoPi * u2);
    }

    float normal(float sigma) { return normal() * sigma; }

private:
    u64 s_;
    float spare_{0.0f};
    bool have_spare_{false};
};

}  // namespace ppa
