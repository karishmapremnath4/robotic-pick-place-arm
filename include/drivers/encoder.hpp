// Wheel encoder driver, over SPI.
//
// Modelled on the AS5047P, a 14 bit absolute magnetic rotary encoder that is
// common on small drives. Two of them, one per wheel, each on its own chip
// select, so each driver instance owns its own bus object.
//
// Two things make this more than a register read.
//
// First the protocol is pipelined. A command goes out in one 16 bit frame and
// the answer comes back during the next one. So a single reading costs two
// transfers, and a driver that reads the same frame it wrote gets the
// previous sample.
//
// Second the part is absolute over one revolution only. It counts 0 to 16383
// and rolls over. Turning that into distance travelled means detecting the
// rollover and accumulating revolutions, which is the unwrapping below. Get
// the sign of a rollover wrong and the wheel appears to jump backwards by
// most of a turn.
#pragma once

#include "hal/bus.hpp"

namespace ppa {

struct EncoderSample {
    // Angle turned SINCE init, unwrapped, growing without bound. Measured
    // from the reference taken at init rather than from the device's own
    // zero, because odometry only ever needs distance travelled and because
    // anchoring to the device zero puts a wrap point in the middle of the
    // measurement. A wheel that starts sitting on that wrap point would
    // otherwise gain a whole spurious revolution on its first move.
    float angle_rad;

    // Raw single turn angle, 0 to 2 pi, before any unwrapping.
    //
    // This is the one an arm actually needs. A wheel cares how far it has
    // rolled, so it wants the accumulated angle above. A joint cares where
    // it IS, in absolute terms, or the kinematics cannot be evaluated at all.
    //
    // Because these joints never travel a full revolution, a single turn
    // absolute part is enough to give absolute position, provided the offset
    // between the encoder's zero and the joint's zero is known. That offset
    // is commissioning data, measured once when the arm is built, and it is
    // why this arm needs no homing sequence at power on.
    float single_turn_rad;
    float velocity_rad_s;  // differentiated between successive reads
    bool valid;
};

class Encoder {
public:
    static constexpr u16 kRegNop = 0x0000;
    static constexpr u16 kRegErrfl = 0x0001;
    static constexpr u16 kRegAngleCom = 0x3FFF;

    static constexpr int kCountsPerRev = 16384;  // 2^14
    static constexpr u16 kAngleMask = 0x3FFF;
    static constexpr u16 kErrorFlagBit = 0x4000;
    static constexpr u16 kParityBit = 0x8000;

    // A jump larger than half a revolution between two reads is read as a
    // rollover rather than as real motion. This is only valid while the wheel
    // turns less than half a revolution per sample interval, which at the
    // 200 Hz read rate and the 18 rad/s wheel limit leaves a wide margin:
    // 18 rad/s over 5 ms is 0.09 rad, well under the 3.14 rad limit.
    static constexpr int kRolloverThreshold = kCountsPerRev / 2;

    Encoder(SpiBus& bus, const Clock& clk) : bus_(bus), clk_(clk) {}

    // Clears any latched error and takes a first reading to establish the
    // reference position. Distance is measured from wherever the wheel
    // happened to be at init, which is all that odometry needs.
    Status init();

    // One reading. Costs two SPI transfers because of the pipelining.
    EncoderSample read();

    u32 error_frames() const { return error_frames_; }
    u32 parity_errors() const { return parity_errors_; }
    u32 rollovers() const { return rollovers_; }

private:
    // Build a read command with even parity over the low 15 bits.
    static u16 read_command(u16 reg);
    // Check even parity across all 16 bits of a response.
    static bool parity_ok(u16 frame);

    Status exchange(u16 out, u16* in);

    SpiBus& bus_;
    const Clock& clk_;

    i32 revolutions_{0};
    u16 last_counts_{0};
    u16 zero_counts_{0};  // reference taken at init
    float last_angle_rad_{0.0f};
    TimeUs last_read_us_{0};
    u32 error_frames_{0};
    u32 parity_errors_{0};
    u32 rollovers_{0};
    bool initialised_{false};
};

}  // namespace ppa
