// Laser distance sensor driver, over UART.
//
// Modelled on the TFmini family, which really exists and really uses this
// frame layout. Nine bytes, little endian, checksummed:
//
//   byte 0   0x59      sync
//   byte 1   0x59      sync
//   byte 2-3 distance  centimetres, unsigned
//   byte 4-5 strength  signal strength, unsigned
//   byte 6-7 temp      raw device temperature
//   byte 8   checksum  low byte of the sum of bytes 0 to 7
//
// The work is not the decode, it is the framing. Bytes arrive in whatever
// clumps the port happens to deliver, so the driver is a state machine that
// hunts for the sync pair, collects a body, and verifies the checksum.
//
// One detail matters more than the rest. When a checksum fails, the driver
// must NOT skip nine bytes and carry on. The corruption may have been a lost
// byte, in which case the real next message started inside what was just
// discarded, and skipping a fixed length guarantees the next frame is
// misaligned too. Every subsequent frame then fails and the link never
// recovers. The fix is to go back to hunting for a sync pair from the byte
// after the failed sync, which resynchronises in at worst one message.
#pragma once

#include "hal/bus.hpp"

namespace ppa {

struct LaserSample {
    float distance_m{0.0f};
    u16 strength{0};
    bool valid{false};
};

// Signal strength below which the reading is meaningless. Real units report
// a number here and it collapses when the target is too far, too dark, or
// the window is dirty, so it is the natural place to reject a bad return
// rather than trusting a distance that happens to be in range.
constexpr u16 kMinStrength = 100;

// The sensor cannot see closer than this or further than that.
constexpr float kLaserMinM = 0.02f;
constexpr float kLaserMaxM = 1.20f;

// No complete frame for this long means the unit has stopped talking.
// Three missed frames at the 50 Hz report rate.
constexpr TimeUs kLaserTimeoutUs = 60000;

class LaserRanger {
public:
    static constexpr u8 kSync = 0x59;
    static constexpr int kFrameLen = 9;

    LaserRanger(UartBus& uart, const Clock& clk) : uart_(uart), clk_(clk) {}

    // Pulls whatever bytes have arrived and feeds them through the state
    // machine. Call at least as often as frames arrive, or the port's buffer
    // overruns and bytes are lost.
    void poll();

    // The most recent good reading, held between frames because a sensor
    // reporting at 50 Hz has nothing new to say most of the time a 1 kHz
    // loop asks it.
    //
    // Once the timeout expires the reading is reported INVALID, not merely
    // old. Holding the last value and still calling it valid is how a dead
    // sensor goes unnoticed: the survey in step 9 swept a whole table with
    // the ranger switched off, binned the same stale distance into every bin,
    // and reported 89 percent coverage of a table it had not seen at all.
    LaserSample sample() const {
        if (timed_out()) {
            LaserSample s = sample_;
            s.valid = false;
            return s;
        }
        return sample_;
    }

    // True once no complete frame has arrived inside the timeout.
    bool timed_out() const;

    u32 frames_ok() const { return frames_ok_; }
    u32 checksum_errors() const { return checksum_errors_; }
    u32 resyncs() const { return resyncs_; }
    u32 weak_returns() const { return weak_returns_; }

private:
    enum class State : u8 { kSync1, kSync2, kBody };

    void feed(u8 byte);
    void commit_frame();

    UartBus& uart_;
    const Clock& clk_;

    State state_{State::kSync1};
    u8 body_[kFrameLen - 2]{};
    int body_len_{0};

    LaserSample sample_{};
    TimeUs last_frame_us_{0};
    bool ever_framed_{false};

    u32 frames_ok_{0};
    u32 checksum_errors_{0};
    u32 resyncs_{0};
    u32 weak_returns_{0};
};

}  // namespace ppa
