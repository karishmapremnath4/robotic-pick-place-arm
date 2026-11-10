#include "drivers/imu.hpp"

namespace ppa {

namespace {
constexpr float kDegToRad = 3.14159265f / 180.0f;
constexpr float kG = 9.80665f;

// Assemble a little endian 16 bit two's complement value.
inline i16 le16(const u8* p) {
    return static_cast<i16>(static_cast<u16>(p[0]) | (static_cast<u16>(p[1]) << 8));
}
}  // namespace

Status Imu::init() {
    u8 id = 0;
    Status st = bus_.read_regs(kI2cAddr, kRegWhoAmI, &id, 1);
    if (st != Status::kOk) {
        ++errors_;
        return st;
    }
    if (id != kWhoAmIValue) {
        ++errors_;
        return Status::kBadId;
    }

    // CTRL3_C: enable register auto increment and block data update, so a
    // burst read cannot tear across a sample boundary.
    st = bus_.write_reg(kI2cAddr, kRegCtrl3C, 0x44);
    if (st != Status::kOk) { ++errors_; return st; }

    // CTRL2_G: 416 Hz output data rate, 2000 dps full scale.
    st = bus_.write_reg(kI2cAddr, kRegCtrl2G, 0x6C);
    if (st != Status::kOk) { ++errors_; return st; }

    // CTRL1_XL: 416 Hz output data rate, 4 g full scale.
    st = bus_.write_reg(kI2cAddr, kRegCtrl1Xl, 0x68);
    if (st != Status::kOk) { ++errors_; return st; }

    initialised_ = true;
    return Status::kOk;
}

ImuSample Imu::read() {
    ImuSample s{0.0f, 0.0f, 0.0f, false};
    if (!initialised_) return s;

    // Ask whether there is anything new before spending a twelve byte burst
    // read on it, and before handing a repeat sample to the estimator.
    u8 status = 0;
    if (bus_.read_regs(kI2cAddr, kRegStatus, &status, 1) != Status::kOk) {
        ++errors_;
        return s;
    }
    if ((status & (kStatusAccelReady | kStatusGyroReady)) == 0) {
        ++stale_reads_;
        return s;
    }

    // Gyro X Y Z then accel X Y Z, six 16 bit values, one transaction.
    u8 raw[12];
    if (bus_.read_regs(kI2cAddr, kRegOutXLG, raw, sizeof(raw)) != Status::kOk) {
        ++errors_;
        return s;
    }

    const i16 gz = le16(&raw[4]);
    const i16 ax = le16(&raw[6]);
    const i16 ay = le16(&raw[8]);

    s.gyro_z = static_cast<float>(gz) * kGyroScaleDpsPerLsb * kDegToRad;
    s.accel_x = static_cast<float>(ax) * kAccelScaleGPerLsb * kG;
    s.accel_y = static_cast<float>(ay) * kAccelScaleGPerLsb * kG;
    s.valid = true;
    return s;
}

}  // namespace ppa
