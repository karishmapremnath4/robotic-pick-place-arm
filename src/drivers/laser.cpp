#include "drivers/laser.hpp"

namespace ppa {

void LaserRanger::poll() {
    // Read in modest chunks rather than one byte at a time, which is what a
    // real driver does when it drains a hardware FIFO or a DMA buffer.
    u8 buf[32];
    for (;;) {
        const u8 n = uart_.read(buf, sizeof(buf));
        if (n == 0) break;
        for (u8 i = 0; i < n; ++i) feed(buf[i]);
        if (n < sizeof(buf)) break;
    }
}

void LaserRanger::feed(u8 byte) {
    switch (state_) {
        case State::kSync1:
            if (byte == kSync) state_ = State::kSync2;
            break;

        case State::kSync2:
            if (byte == kSync) {
                state_ = State::kBody;
                body_len_ = 0;
            } else {
                // Not a sync pair. This byte might itself be the start of a
                // real pair, so it is re-examined rather than dropped.
                state_ = (byte == kSync) ? State::kSync2 : State::kSync1;
            }
            break;

        case State::kBody:
            body_[body_len_++] = byte;
            if (body_len_ == kFrameLen - 2) {
                commit_frame();
                state_ = State::kSync1;
                body_len_ = 0;
            }
            break;
    }
}

void LaserRanger::commit_frame() {
    // Checksum covers the two sync bytes and the first six body bytes.
    u32 sum = static_cast<u32>(kSync) + static_cast<u32>(kSync);
    for (int i = 0; i < 6; ++i) sum += body_[i];
    const u8 expect = static_cast<u8>(sum & 0xFF);

    if (expect != body_[6]) {
        ++checksum_errors_;
        ++resyncs_;
        // Deliberately no skipping: state has already gone back to hunting
        // for a sync pair, so if a byte was lost the next real frame is found
        // within one message instead of the link staying misaligned forever.
        return;
    }

    const u16 dist_cm = static_cast<u16>(static_cast<u16>(body_[0]) |
                                         (static_cast<u16>(body_[1]) << 8));
    const u16 strength = static_cast<u16>(static_cast<u16>(body_[2]) |
                                          (static_cast<u16>(body_[3]) << 8));

    ++frames_ok_;
    last_frame_us_ = clk_.now_us();
    ever_framed_ = true;

    // Half a centimetre is added back on purpose.
    //
    // The device reports whole centimetres and truncates rather than rounds,
    // so a true 9.37 cm arrives as 9. Taking that at face value biases every
    // distance 5 mm short, and anything computing a surface height from it
    // biases 5 mm high, systematically and in one direction. The survey in
    // step 9 measured box tops 5.3 mm above where they are, consistently,
    // which no amount of averaging removes because it is not noise.
    //
    // Adding half a count puts the estimate at the centre of the interval
    // the device is actually reporting, which is the best available answer
    // and is unbiased.
    const float d = (static_cast<float>(dist_cm) + 0.5f) * 0.01f;

    // A weak return means the unit could not see anything useful. The
    // distance field still holds a number, and trusting it is how a dirty
    // window turns into a crash.
    if (strength < kMinStrength || d < kLaserMinM || d > kLaserMaxM) {
        ++weak_returns_;
        sample_.strength = strength;
        sample_.valid = false;
        return;
    }

    sample_.distance_m = d;
    sample_.strength = strength;
    sample_.valid = true;
}

bool LaserRanger::timed_out() const {
    if (!ever_framed_) return false;  // nothing to time out from yet
    return (clk_.now_us() - last_frame_us_) > kLaserTimeoutUs;
}

}  // namespace ppa
