// Gyroscope and accelerometer driver, over I2C.
//
// Register map is modelled on the ST LSM6DSL, a part that really exists and is
// common on robot boards. Bring up is the standard sequence: confirm the
// device identity, configure output data rate and full scale, then burst read
// the measurement registers.
//
// Raw registers are 16 bit two's complement, little endian, and must be scaled
// by the full scale setting to get physical units.
#pragma once

#include "hal/bus.hpp"

namespace ppa {

struct ImuSample {
    float gyro_z;    // rad/s about the vertical axis
    float accel_x;   // m/s^2 forward
    float accel_y;   // m/s^2 left
    bool valid;
};

class Imu {
public:
    // Device registers.
    static constexpr u8 kI2cAddr = 0x6A;
    static constexpr u8 kRegWhoAmI = 0x0F;
    static constexpr u8 kWhoAmIValue = 0x6A;
    static constexpr u8 kRegCtrl1Xl = 0x10;  // accelerometer
    static constexpr u8 kRegCtrl2G = 0x11;   // gyroscope
    static constexpr u8 kRegCtrl3C = 0x12;   // common
    static constexpr u8 kRegStatus = 0x1E;   // STATUS_REG
    static constexpr u8 kRegOutXLG = 0x22;   // gyro X low, 12 bytes to accel

    // STATUS_REG data available flags. Reading the output registers clears
    // them, which is how the device says "this sample is new".
    static constexpr u8 kStatusAccelReady = 0x01;  // XLDA
    static constexpr u8 kStatusGyroReady = 0x02;   // GDA

    // Full scale settings chosen at init.
    //   gyro  2000 dps  ->  70 mdps per count
    //   accel    4 g    ->  0.122 mg per count
    static constexpr float kGyroScaleDpsPerLsb = 0.070f;
    static constexpr float kAccelScaleGPerLsb = 0.000122f;

    // Saturation limit implied by the 2000 dps full scale.
    static constexpr float kGyroMaxRadS = 2000.0f * 3.14159265f / 180.0f;

    explicit Imu(I2cBus& bus) : bus_(bus) {}

    // Confirms identity and writes the configuration registers.
    Status init();

    // Burst reads gyro and accelerometer in one transaction.
    //
    // Returns valid = false when the device has no new sample. This matters
    // more than it looks: the part converts at 416 Hz while the estimator
    // runs at 1 kHz, so most calls have nothing new to offer. Handing the
    // same sample to a Kalman filter three times makes it believe it has
    // three independent measurements when it has one, and the filter becomes
    // confident on the strength of data it never received.
    //
    // So the driver checks STATUS_REG first and says so.
    ImuSample read();

    // Samples skipped because the device had not converted a new one. Worth
    // counting: the ratio against the poll rate should match the configured
    // output data rate, and if it does not then something is wrong with the
    // configuration rather than with the filter.
    u32 stale_reads() const { return stale_reads_; }

    u32 error_count() const { return errors_; }

private:
    I2cBus& bus_;
    u32 errors_{0};
    u32 stale_reads_{0};
    bool initialised_{false};
};

}  // namespace ppa
