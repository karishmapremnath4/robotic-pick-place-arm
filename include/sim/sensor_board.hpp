// Simulated sensor board for the PPA-100 cell.
//
// This is the layer that makes the project worth doing. The firmware talks to
// these objects through the same I2cBus, SpiBus and CanBus interfaces it
// would use on a real board, and behind them sit device models that hold
// their own registers and follow their own datasheets.
//
// So the IMU driver genuinely writes a configuration byte, and this code
// genuinely decodes the full scale bits out of it and scales its output to
// match. Configure it wrongly and the numbers come back wrong, exactly as
// they would on a bench.
//
// Devices:
//   LSM6DSL on the tool          I2C, address 0x6A
//   AS5047P per joint, two       SPI, one chip select each
//   two servo drives, gripper,
//   and a safety relay           CAN, four nodes
//
// Faults are injected here, at the device. Never in the estimator.
#pragma once

#include "config.hpp"
#include "drivers/can_node.hpp"
#include "ctrl/current_loop.hpp"
#include "drivers/laser.hpp"
#include "hal/bus.hpp"
#include "rtos/ring.hpp"
#include "sim/faults.hpp"
#include "sim/world.hpp"

namespace ppa {

// Monotonic clock driven by the simulation rather than the wall clock.
// Advancing time explicitly is what makes a run repeatable and lets the
// simulation run faster than real time when nobody is watching.
class SimClock final : public Clock {
public:
    TimeUs now_us() const override { return t_; }
    void advance_us(TimeUs dt) { t_ += dt; }

private:
    TimeUs t_{0};
};

// ---------------------------------------------------------------------------
// LSM6DSL on the tool, over I2C
// ---------------------------------------------------------------------------
class SimToolImuI2c final : public I2cBus {
public:
    SimToolImuI2c(const World& w, const Faults& f, const SimClock& clk, Rng& rng)
        : w_(w), f_(f), clk_(clk), rng_(rng) {}

    Status write_reg(u8 addr, u8 reg, u8 value) override;
    Status read_regs(u8 addr, u8 reg, u8* dst, u8 len) override;

    // The device refreshes its output registers at its configured data rate,
    // not on demand, so a driver polling faster than that legitimately sees
    // the same sample twice. Drivers that assume every read is fresh get
    // caught here rather than on hardware.
    void update();

    u32 nack_count() const { return nacks_; }

private:
    void refresh_outputs();

    static constexpr u8 kAddr = 0x6A;
    static constexpr int kRegCount = 0x80;

    const World& w_;
    const Faults& f_;
    const SimClock& clk_;
    Rng& rng_;

    u8 regs_[kRegCount]{};
    TimeUs last_refresh_us_{0};
    u32 odr_hz_{0};
    float gyro_dps_per_lsb_{0.0f};
    float accel_g_per_lsb_{0.0f};
    u32 nacks_{0};
    bool configured_{false};
};

// ---------------------------------------------------------------------------
// AS5047P per joint, over SPI
// ---------------------------------------------------------------------------
//
// 14 bit single turn absolute encoder. Two of them, one per joint, each with
// its own chip select, so each gets its own bus object and no arbitration is
// needed inside a transfer.
//
// The protocol is two framed transfers: a command goes out in one 16 bit
// exchange and the answer arrives in the next. A driver that expects its
// answer in the same frame reads one sample stale.
class SimJointEncoderSpi final : public SpiBus {
public:
    SimJointEncoderSpi(const World& w, const Faults& f, int joint, Rng& rng)
        : w_(w), f_(f), joint_(joint), rng_(rng) {}

    Status transfer(const u8* tx, u8* rx, u8 len) override;

    static constexpr u16 kRegAngleCom = 0x3FFF;
    static constexpr u16 kRegErrfl = 0x0001;
    static constexpr int kCountsPerRev = 16384;

private:
    static u16 with_parity(u16 v);
    static bool parity_ok(u16 v);

    const World& w_;
    const Faults& f_;
    int joint_;  // 0 or 1
    Rng& rng_;

    u16 pending_response_{0};
    bool have_pending_{false};
    // Latched angle for the stuck fault: the device keeps clocking out its
    // last conversion, which is valid data that has simply stopped changing.
    float stuck_angle_rad_{0.0f};
    bool stuck_latched_{false};
};

// ---------------------------------------------------------------------------
// Laser distance sensor, over UART
// ---------------------------------------------------------------------------
//
// Emits its nine byte frames one byte at a time at the line rate, not as a
// block. That matters: at 115200 baud a frame takes 0.78 ms to arrive, so a
// firmware polling at 1 kHz genuinely catches half finished frames and the
// driver's state machine genuinely has to carry state across poll
// boundaries. Delivering whole frames atomically would make the hardest part
// of a UART driver untestable.
class SimLaserUart final : public UartBus {
public:
    SimLaserUart(const World& w, const Faults& f, const SimClock& clk, Rng& rng)
        : w_(w), f_(f), clk_(clk), rng_(rng) {}

    u8 read(u8* dst, u8 len) override;
    Status write(const u8* src, u8 len) override;

    void update();

    // 115200 baud, 8N1, so ten bit times per byte: 86.8 us each.
    static constexpr TimeUs kByteTimeUs = 87;
    static constexpr TimeUs kReportPeriodUs = 20000;  // 50 Hz
    static constexpr int kFrameLen = 9;
    static constexpr int kRxCapacity = 256;

    u32 frames_sent() const { return frames_sent_; }
    u32 bytes_corrupted() const { return corrupted_; }

private:
    void build_frame();

    const World& w_;
    const Faults& f_;
    const SimClock& clk_;
    Rng& rng_;

    Ring<u8, kRxCapacity> rx_{};
    u8 frame_[kFrameLen]{};
    int frame_pos_{kFrameLen};  // kFrameLen means nothing in flight
    TimeUs last_byte_us_{0};
    TimeUs last_report_us_{0};
    u32 frames_sent_{0};
    u32 corrupted_{0};
};

// ---------------------------------------------------------------------------
// CAN: two servo drives, the gripper, and a safety relay
// ---------------------------------------------------------------------------
class SimCan final : public CanBus {
public:
    SimCan(World& w, const Faults& f, const SimClock& clk, Rng& rng)
        : w_(w), f_(f), clk_(clk), rng_(rng) {}

    Status send(const CanFrame& frame) override;
    bool receive(CanFrame* out) override;

    // Generates the periodic traffic the four nodes would produce, and
    // applies whatever the firmware last commanded to the cell.
    void update();

    // Depth chosen against the worst case traffic: two drives at 1 kHz, the
    // gripper at 100 Hz and four heartbeats at 10 Hz is about 2140 frames a
    // second, so a firmware polling at 1 kHz drains two or three per pass.
    // 64 leaves room for the receive task to be late by tens of periods.
    static constexpr int kRxCapacity = 64;

    u32 tx_count() const { return tx_; }
    u32 rx_dropped() const { return rx_.dropped(); }
    u32 rx_high_water() const { return rx_.high_water(); }

private:
    void emit_drive_status();
    void run_current_loops();
    void emit_gripper_status();
    void emit_heartbeats();
    bool node_alive(u8 node) const;

    World& w_;
    const Faults& f_;
    const SimClock& clk_;
    Rng& rng_;

    Ring<CanFrame, kRxCapacity> rx_{};
    TimeUs last_drive_us_{0};
    TimeUs last_grip_us_{0};
    TimeUs last_heartbeat_us_{0};

    // The drives' own current loops, closed inside the drive at 10 kHz where
    // the bus cannot reach. Commands from the bus are current setpoints.
    static constexpr TimeUs kCurrentLoopPeriodUs = 100;  // 10 kHz
    CurrentLoop cl_[2]{};
    bool cl_ready_{false};
    TimeUs last_current_us_{0};
    float cmd_i1_{0.0f};
    float cmd_i2_{0.0f};
    u32 tx_{0};
};

// Measurement noise the models actually apply. The estimator is given its own
// numbers, deliberately not identical to these, because on real hardware you
// never know the true noise and tuning to the exact value fools you.
struct SensorNoise {
    float gyro_rad_s{0.004f};
    float accel_m_s2{0.08f};
    float encoder_counts{1.5f};
    float drive_pos_rad{0.0004f};
    float drive_vel_rad_s{0.010f};
    float drive_current_a{0.015f};
    float clamp_n{0.15f};
    float laser_cm{0.6f};
};
extern SensorNoise g_noise;

// Everything wired together, which is what the application holds.
struct SensorBoard {
    SensorBoard(World& w, const Faults& f, const SimClock& clk, Rng& rng)
        : imu_bus(w, f, clk, rng),
          enc1_bus(w, f, 0, rng),
          enc2_bus(w, f, 1, rng),
          can_bus(w, f, clk, rng),
          laser_bus(w, f, clk, rng) {}

    void update() {
        imu_bus.update();
        can_bus.update();
        laser_bus.update();
    }

    SimToolImuI2c imu_bus;
    SimJointEncoderSpi enc1_bus;
    SimJointEncoderSpi enc2_bus;
    SimCan can_bus;
    SimLaserUart laser_bus;
};

}  // namespace ppa
