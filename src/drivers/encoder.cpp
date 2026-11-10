#include "drivers/encoder.hpp"

namespace ppa {

namespace {
constexpr float kTwoPi = 6.28318530717959f;
constexpr float kRadPerCount =
    kTwoPi / static_cast<float>(Encoder::kCountsPerRev);

// Even parity over the low 15 bits, folded down with shifts. No loop and no
// lookup table, which keeps it cheap enough to run on every frame.
u16 parity_of_low15(u16 v) {
    u16 p = static_cast<u16>(v & 0x7FFF);
    p ^= static_cast<u16>(p >> 8);
    p ^= static_cast<u16>(p >> 4);
    p ^= static_cast<u16>(p >> 2);
    p ^= static_cast<u16>(p >> 1);
    return static_cast<u16>(p & 1u);
}
}  // namespace

u16 Encoder::read_command(u16 reg) {
    const u16 v = static_cast<u16>((reg & kAngleMask) | 0x4000);  // read bit
    return static_cast<u16>(v | (parity_of_low15(v) << 15));
}

bool Encoder::parity_ok(u16 frame) {
    // Even parity over all 16 bits, so the fold of the whole word must be 0.
    u16 p = frame;
    p ^= static_cast<u16>(p >> 8);
    p ^= static_cast<u16>(p >> 4);
    p ^= static_cast<u16>(p >> 2);
    p ^= static_cast<u16>(p >> 1);
    return (p & 1u) == 0u;
}

Status Encoder::exchange(u16 out, u16* in) {
    u8 tx[2] = {static_cast<u8>(out >> 8), static_cast<u8>(out & 0xFF)};
    u8 rx[2] = {0, 0};
    const Status st = bus_.transfer(tx, rx, 2);
    if (st != Status::kOk) return st;
    *in = static_cast<u16>((static_cast<u16>(rx[0]) << 8) |
                           static_cast<u16>(rx[1]));
    return Status::kOk;
}

Status Encoder::init() {
    // Reading ERRFL clears it. The response to this command arrives in the
    // next frame, so this first exchange is only there to push the command
    // out; its returned data is meaningless and deliberately ignored.
    u16 discard = 0;
    Status st = exchange(read_command(kRegErrfl), &discard);
    if (st != Status::kOk) return st;

    // Now ask for the angle, and collect the ERRFL answer that arrives with
    // this frame.
    st = exchange(read_command(kRegAngleCom), &discard);
    if (st != Status::kOk) return st;

    // And collect the angle itself, which establishes the reference.
    u16 frame = 0;
    st = exchange(read_command(kRegAngleCom), &frame);
    if (st != Status::kOk) return st;

    if (!parity_ok(frame)) {
        ++parity_errors_;
        return Status::kNack;
    }
    if (frame & kErrorFlagBit) {
        ++error_frames_;
        return Status::kNack;
    }

    last_counts_ = static_cast<u16>(frame & kAngleMask);
    zero_counts_ = last_counts_;
    revolutions_ = 0;
    last_angle_rad_ = 0.0f;
    last_read_us_ = clk_.now_us();
    initialised_ = true;
    return Status::kOk;
}

EncoderSample Encoder::read() {
    EncoderSample s{last_angle_rad_, 0.0f, 0.0f, false};
    if (!initialised_) return s;

    // Two transfers: the first carries the command, the second brings the
    // answer while carrying the next command.
    u16 frame = 0;
    if (exchange(read_command(kRegAngleCom), &frame) != Status::kOk) return s;
    if (exchange(read_command(kRegAngleCom), &frame) != Status::kOk) return s;

    if (!parity_ok(frame)) {
        ++parity_errors_;
        return s;
    }
    if (frame & kErrorFlagBit) {
        ++error_frames_;
        return s;
    }

    const u16 counts = static_cast<u16>(frame & kAngleMask);

    // Unwrap. The raw difference is taken as a signed value first, then a
    // magnitude over half a revolution is reinterpreted as a rollover in the
    // other direction.
    int delta = static_cast<int>(counts) - static_cast<int>(last_counts_);
    if (delta > kRolloverThreshold) {
        delta -= kCountsPerRev;
        --revolutions_;
        ++rollovers_;
    } else if (delta < -kRolloverThreshold) {
        delta += kCountsPerRev;
        ++revolutions_;
        ++rollovers_;
    }
    last_counts_ = counts;

    const float angle = (static_cast<float>(revolutions_) *
                         static_cast<float>(kCountsPerRev) +
                         static_cast<float>(counts) -
                         static_cast<float>(zero_counts_)) * kRadPerCount;

    const TimeUs now = clk_.now_us();
    const TimeUs dt_us = now - last_read_us_;

    // A zero or absurdly small interval would divide the count noise by
    // nearly nothing and produce a wild velocity, so the sample is reported
    // as position only in that case.
    if (dt_us >= 100) {
        const float dt = static_cast<float>(dt_us) * 1e-6f;
        s.velocity_rad_s = (angle - last_angle_rad_) / dt;
        last_read_us_ = now;
        last_angle_rad_ = angle;
    } else {
        s.velocity_rad_s = 0.0f;
    }

    s.angle_rad = angle;
    s.single_turn_rad = static_cast<float>(counts) * kRadPerCount;
    s.valid = true;
    return s;
}

}  // namespace ppa
