#include "sim/sensor_board.hpp"

#include <cmath>
#include <cstring>

namespace ppa {

SensorNoise g_noise{};

namespace {

constexpr float kG = 9.80665f;
constexpr float kDegToRad = 3.14159265358979f / 180.0f;
constexpr float kTwoPi = 6.28318530717959f;

i16 sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return static_cast<i16>(v);
}

u16 satu16(float v) {
    if (v < 0.0f) return 0;
    if (v > 65535.0f) return 65535;
    return static_cast<u16>(v);
}

void put_le16(u8* p, i16 v) {
    p[0] = static_cast<u8>(static_cast<u16>(v) & 0xFF);
    p[1] = static_cast<u8>((static_cast<u16>(v) >> 8) & 0xFF);
}

void put_leu16(u8* p, u16 v) {
    p[0] = static_cast<u8>(v & 0xFF);
    p[1] = static_cast<u8>((v >> 8) & 0xFF);
}

i16 get_le16(const u8* p) {
    return static_cast<i16>(static_cast<u16>(p[0]) |
                            (static_cast<u16>(p[1]) << 8));
}

// LSM6DSL output data rate, from the ODR bits of CTRL1_XL and CTRL2_G.
u32 decode_odr_hz(u8 bits) {
    switch (bits & 0x0F) {
        case 0x0: return 0;  // powered down
        case 0x1: return 13;
        case 0x2: return 26;
        case 0x3: return 52;
        case 0x4: return 104;
        case 0x5: return 208;
        case 0x6: return 416;
        case 0x7: return 833;
        case 0x8: return 1660;
        case 0x9: return 3330;
        case 0xA: return 6660;
        default: return 0;
    }
}

// FS_G bits of CTRL2_G, in degrees per second per count.
float decode_gyro_scale_dps(u8 fs_bits) {
    switch (fs_bits & 0x03) {
        case 0x0: return 0.00875f;  //  245 dps
        case 0x1: return 0.01750f;  //  500 dps
        case 0x2: return 0.03500f;  // 1000 dps
        default: return 0.07000f;   // 2000 dps
    }
}

// FS_XL bits of CTRL1_XL. The ordering is not monotonic; that is how the part
// is specified and getting it wrong is a classic scaling bug.
float decode_accel_scale_g(u8 fs_bits) {
    switch (fs_bits & 0x03) {
        case 0x0: return 0.000061f;  //  2 g
        case 0x1: return 0.000488f;  // 16 g
        case 0x2: return 0.000122f;  //  4 g
        default: return 0.000244f;   //  8 g
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// LSM6DSL on the tool, over I2C
// ---------------------------------------------------------------------------

Status SimToolImuI2c::write_reg(u8 addr, u8 reg, u8 value) {
    if (addr != kAddr) return Status::kNack;
    if (f_.imu_nack) { ++nacks_; return Status::kNack; }
    if (reg >= kRegCount) return Status::kRangeError;

    regs_[reg] = value;

    if (reg == 0x11) {  // CTRL2_G
        odr_hz_ = decode_odr_hz(static_cast<u8>(value >> 4));
        gyro_dps_per_lsb_ = decode_gyro_scale_dps(static_cast<u8>(value >> 2));
    } else if (reg == 0x10) {  // CTRL1_XL
        accel_g_per_lsb_ = decode_accel_scale_g(static_cast<u8>(value >> 2));
    }

    configured_ = (odr_hz_ > 0) && (gyro_dps_per_lsb_ > 0.0f) &&
                  (accel_g_per_lsb_ > 0.0f);
    return Status::kOk;
}

Status SimToolImuI2c::read_regs(u8 addr, u8 reg, u8* dst, u8 len) {
    if (addr != kAddr) return Status::kNack;
    if (f_.imu_nack) { ++nacks_; return Status::kNack; }
    if (reg >= kRegCount) return Status::kRangeError;
    if (static_cast<int>(reg) + static_cast<int>(len) > kRegCount)
        return Status::kRangeError;

    // WHO_AM_I answers before any configuration, which is what makes it
    // usable for identifying the part during bring up.
    if (reg == 0x0F) {
        for (u8 i = 0; i < len; ++i) dst[i] = kAddr;  // 0x6A
        return Status::kOk;
    }

    // Reading the output registers clears the data available flags. That is
    // what makes STATUS_REG meaningful: the firmware cannot see the same
    // sample twice and be told it is new.
    if (reg >= 0x22 && reg < 0x2E) {
        regs_[0x1E] = 0;
    }

    // CTRL3_C bit 2 is IF_INC, register auto increment. With it clear the
    // address pointer does not advance, so a burst read returns the same byte
    // repeatedly. A driver that forgets to set it is caught here.
    const bool auto_increment = (regs_[0x12] & 0x04) != 0;
    if (!auto_increment) {
        for (u8 i = 0; i < len; ++i) dst[i] = regs_[reg];
        return Status::kOk;
    }

    std::memcpy(dst, &regs_[reg], len);
    return Status::kOk;
}

void SimToolImuI2c::update() {
    if (!configured_ || odr_hz_ == 0) return;

    const TimeUs period_us = 1000000ull / odr_hz_;
    if (clk_.now_us() - last_refresh_us_ < period_us) return;
    last_refresh_us_ = clk_.now_us();

    // A stuck device keeps clocking out its last conversion. The registers
    // are simply not updated, which is what the fault looks like from the
    // firmware side: valid reads, plausible values, frozen data.
    if (f_.imu_stuck) return;

    refresh_outputs();
}

void SimToolImuI2c::refresh_outputs() {
    const float nm = f_.imu_noise_mult;

    // The tool frame gyro sees the wrist rotation rate, which for a planar
    // two link arm is qd1 + qd2.
    const float gyro_z = static_cast<float>(w_.tool_rate()) +
                         rng_.normal(g_noise.gyro_rad_s * nm);

    // Out of plane rates are noise only on a planar arm.
    const float gyro_x = rng_.normal(g_noise.gyro_rad_s * nm);
    const float gyro_y = rng_.normal(g_noise.gyro_rad_s * nm);

    // Specific force, already in the tool frame and already including
    // gravity, straight from the plant.
    const arm::Vec2 sf = w_.tool_specific_force();
    const float accel_x = static_cast<float>(sf[0]) +
                          rng_.normal(g_noise.accel_m_s2 * nm);
    const float accel_y = static_cast<float>(sf[1]) +
                          rng_.normal(g_noise.accel_m_s2 * nm);
    // Nothing accelerates the tool out of plane, so the third axis is noise.
    const float accel_z = rng_.normal(g_noise.accel_m_s2 * nm);

    const float gyro_lsb = gyro_dps_per_lsb_ * kDegToRad;
    const float accel_lsb = accel_g_per_lsb_ * kG;

    put_le16(&regs_[0x22], sat16(gyro_x / gyro_lsb));
    put_le16(&regs_[0x24], sat16(gyro_y / gyro_lsb));
    put_le16(&regs_[0x26], sat16(gyro_z / gyro_lsb));
    put_le16(&regs_[0x28], sat16(accel_x / accel_lsb));
    put_le16(&regs_[0x2A], sat16(accel_y / accel_lsb));
    put_le16(&regs_[0x2C], sat16(accel_z / accel_lsb));

    // STATUS_REG: both XLDA and GDA, a new sample is available.
    regs_[0x1E] = 0x03;
}

// ---------------------------------------------------------------------------
// AS5047P per joint, over SPI
// ---------------------------------------------------------------------------

u16 SimJointEncoderSpi::with_parity(u16 v) {
    const u16 x = static_cast<u16>(v & 0x7FFF);
    u16 p = x;
    p ^= static_cast<u16>(p >> 8);
    p ^= static_cast<u16>(p >> 4);
    p ^= static_cast<u16>(p >> 2);
    p ^= static_cast<u16>(p >> 1);
    return static_cast<u16>(x | ((p & 1u) << 15));
}

bool SimJointEncoderSpi::parity_ok(u16 v) {
    u16 p = v;
    p ^= static_cast<u16>(p >> 8);
    p ^= static_cast<u16>(p >> 4);
    p ^= static_cast<u16>(p >> 2);
    p ^= static_cast<u16>(p >> 1);
    return (p & 1u) == 0u;
}

Status SimJointEncoderSpi::transfer(const u8* tx, u8* rx, u8 len) {
    if (len != 2) return Status::kRangeError;

    const u16 cmd = static_cast<u16>((static_cast<u16>(tx[0]) << 8) |
                                     static_cast<u16>(tx[1]));

    // The answer to the PREVIOUS command comes out during this frame.
    const u16 out = have_pending_ ? pending_response_ : 0x0000;
    rx[0] = static_cast<u8>(out >> 8);
    rx[1] = static_cast<u8>(out & 0xFF);

    bool frame_error = !parity_ok(cmd);
    const bool is_read = (cmd & 0x4000) != 0;
    const u16 addr = static_cast<u16>(cmd & 0x3FFF);

    u16 data = 0;
    if (f_.enc_error_frames) {
        frame_error = true;
    } else if (is_read && addr == kRegAngleCom) {
        // The joint angle plus the commissioning offset, wrapped into one
        // revolution. That wrap is the whole reason the firmware has to
        // subtract the offset and rewrap rather than just scaling counts.
        float ang;
        const bool stuck = (f_.enc_stuck_joint == joint_ + 1);
        if (stuck) {
            if (!stuck_latched_) {
                stuck_angle_rad_ = static_cast<float>(w_.q()[joint_]);
                stuck_latched_ = true;
            }
            ang = stuck_angle_rad_;
        } else {
            stuck_latched_ = false;
            ang = static_cast<float>(w_.q()[joint_]);
        }

        float shaft = ang + static_cast<float>(kEncoderZeroOffsetRad[joint_]);
        shaft = std::fmod(shaft, kTwoPi);
        if (shaft < 0.0f) shaft += kTwoPi;

        float counts = shaft * (static_cast<float>(kCountsPerRev) / kTwoPi) +
                       rng_.normal(g_noise.encoder_counts);
        if (counts < 0.0f) counts += static_cast<float>(kCountsPerRev);
        data = static_cast<u16>(static_cast<int>(counts) % kCountsPerRev);
    }

    u16 resp = static_cast<u16>(data & 0x3FFF);
    if (frame_error) resp |= 0x4000;
    pending_response_ = with_parity(resp);
    have_pending_ = true;
    return Status::kOk;
}

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------

bool SimCan::node_alive(u8 node) const {
    if (f_.can_bus_off) return false;
    if (f_.can_silent_node == node) return false;
    return true;
}

Status SimCan::send(const CanFrame& frame) {
    // Bus off means the controller has counted too many errors and detached
    // itself. Transmits fail outright until it is reset.
    if (f_.can_bus_off) return Status::kTimeout;

    ++tx_;
    switch (frame.id) {
        case canid::kCmdDrive1:
            if (frame.dlc < 2) return Status::kRangeError;
            cmd_i1_ = static_cast<float>(get_le16(&frame.data[0])) * kCurrentLsbA;
            break;
        case canid::kCmdDrive2:
            if (frame.dlc < 2) return Status::kRangeError;
            cmd_i2_ = static_cast<float>(get_le16(&frame.data[0])) * kCurrentLsbA;
            break;
        case canid::kCmdGripper:
            if (frame.dlc < 1) return Status::kRangeError;
            w_.command_grip(frame.data[0] != 0);
            break;
        default:
            return Status::kBadId;
    }

    // A silent drive still accepts commands in this model: it has stopped
    // transmitting, not stopped listening. That asymmetry matters, because
    // the arm keeps moving while the firmware is blind to its state.
    //
    // Nothing is applied to the plant here. The setpoint is only a request;
    // the drive's own current loop turns it into a voltage, and it does that
    // on its own 10 kHz schedule rather than whenever a frame happens to
    // arrive.
    return Status::kOk;
}

void SimCan::run_current_loops() {
    if (!cl_ready_) {
        cl_[0].configure(0);
        cl_[1].configure(1);
        cl_ready_ = true;
    }

    const float dt = static_cast<float>(kCurrentLoopPeriodUs) * 1e-6f;
    const arm::Vec2 i_meas = w_.current();
    const arm::Vec2 qd = w_.qd();

    // The drive measures its own current and its own shaft speed directly,
    // on its own board. No bus involved, which is the entire reason it can
    // run this fast.
    const float v1 = cl_[0].update(cmd_i1_, static_cast<float>(i_meas[0]),
                                   static_cast<float>(qd[0]), dt);
    const float v2 = cl_[1].update(cmd_i2_, static_cast<float>(i_meas[1]),
                                   static_cast<float>(qd[1]), dt);
    w_.set_voltage(v1, v2);
}

bool SimCan::receive(CanFrame* out) {
    if (f_.can_bus_off) return false;
    return rx_.pop(out);
}

void SimCan::update() {
    const TimeUs now = clk_.now_us();

    // The current loops keep running even with the bus off. A drive that
    // loses communications holds its last setpoint; it does not drop the
    // arm. Making that explicit matters, because "bus off" must not silently
    // become "brakes on" in the simulation when it would not be on real
    // hardware.
    if (now - last_current_us_ >= kCurrentLoopPeriodUs) {
        last_current_us_ = now;
        run_current_loops();
    }

    if (f_.can_bus_off) return;

    if (now - last_drive_us_ >= 1000) {  // 1 kHz
        last_drive_us_ = now;
        emit_drive_status();
    }
    if (now - last_grip_us_ >= 10000) {  // 100 Hz
        last_grip_us_ = now;
        emit_gripper_status();
    }
    if (now - last_heartbeat_us_ >= 100000) {  // 10 Hz
        last_heartbeat_us_ = now;
        emit_heartbeats();
    }
}

void SimCan::emit_drive_status() {
    for (int j = 0; j < 2; ++j) {
        const u8 node = (j == 0) ? kNodeDrive1 : kNodeDrive2;
        if (!node_alive(node)) continue;

        // The drive reports from its OWN encoder, independent of the SPI
        // encoders the controller reads. Same quantity, different sensor, so
        // different noise and different failure modes. That independence is
        // what makes cross checking them worth anything.
        const float pos = static_cast<float>(w_.q()[j]) +
                          rng_.normal(g_noise.drive_pos_rad);
        const float vel = static_cast<float>(w_.qd()[j]) +
                          rng_.normal(g_noise.drive_vel_rad_s);
        const float cur = static_cast<float>(w_.current()[j]) +
                          rng_.normal(g_noise.drive_current_a) +
                          f_.current_bias_a[j];

        CanFrame fr{};
        fr.id = (j == 0) ? canid::kStatusDrive1 : canid::kStatusDrive2;
        fr.dlc = 6;
        put_le16(&fr.data[0], sat16(pos / kPosLsbRad));
        put_le16(&fr.data[2], sat16(vel / kVelLsbRadS));
        put_le16(&fr.data[4], sat16(cur / kCurrentLsbA));
        rx_.push(fr);
    }
}

void SimCan::emit_gripper_status() {
    if (!node_alive(kNodeGripper)) return;

    u8 state = 0;
    switch (w_.grip_state()) {
        case GripState::kOpen: state = static_cast<u8>(GripReport::kOpen); break;
        case GripState::kClosing: state = static_cast<u8>(GripReport::kClosing); break;
        case GripState::kHolding: state = static_cast<u8>(GripReport::kHolding); break;
        case GripState::kOpening: state = static_cast<u8>(GripReport::kOpening); break;
    }

    const float clamp = static_cast<float>(w_.grip_force_n()) +
                        rng_.normal(g_noise.clamp_n);

    CanFrame fr{};
    fr.id = canid::kStatusGripper;
    fr.dlc = 3;
    fr.data[0] = state;
    put_leu16(&fr.data[1], satu16(clamp / kClampLsbN));
    rx_.push(fr);
}

// ---------------------------------------------------------------------------
// Laser distance sensor, over UART
// ---------------------------------------------------------------------------

void SimLaserUart::build_frame() {
    const arm::Vec2 tip = w_.tip();
    const double beam_x = tip[0] + static_cast<double>(kLaserOffsetXM);
    const double d = w_.distance_below(beam_x, tip[1], f_);

    float d_cm = static_cast<float>(d) * 100.0f +
                 rng_.normal(g_noise.laser_cm);
    if (d_cm < 0.0f) d_cm = 0.0f;

    // Signal strength collapses when the target is out of range. Real units
    // behave this way, and it is what lets a driver reject a distance that
    // happens to look plausible.
    u16 strength = 1200;
    if (d > static_cast<double>(kLaserMaxM) ||
        d < static_cast<double>(kLaserMinM)) {
        strength = 20;
    }

    const u16 dist_cm = static_cast<u16>(d_cm);
    const u16 temp_raw = 5400;  // arbitrary but constant, as the part reports

    frame_[0] = 0x59;
    frame_[1] = 0x59;
    frame_[2] = static_cast<u8>(dist_cm & 0xFF);
    frame_[3] = static_cast<u8>(dist_cm >> 8);
    frame_[4] = static_cast<u8>(strength & 0xFF);
    frame_[5] = static_cast<u8>(strength >> 8);
    frame_[6] = static_cast<u8>(temp_raw & 0xFF);
    frame_[7] = static_cast<u8>(temp_raw >> 8);

    u32 sum = 0;
    for (int i = 0; i < 8; ++i) sum += frame_[i];
    frame_[8] = static_cast<u8>(sum & 0xFF);

    // Line noise. One byte in this frame gets a bit flipped, which is what
    // a marginal cable or a bad ground actually does. The checksum has to
    // catch it and the driver has to resynchronise.
    if (f_.laser_dropouts && rng_.uniform() < 0.25f) {
        const int which = static_cast<int>(rng_.uniform() * 8.0f) % 8;
        frame_[which] ^= static_cast<u8>(1u << (static_cast<int>(
            rng_.uniform() * 8.0f) % 8));
        ++corrupted_;
    }

    frame_pos_ = 0;
    ++frames_sent_;
}

void SimLaserUart::update() {
    const TimeUs now = clk_.now_us();
    if (f_.laser_silent) return;

    // A new reading every 20 ms, but only once the previous frame has
    // finished going out. A real unit does not overlap its own frames.
    if (frame_pos_ >= kFrameLen && now - last_report_us_ >= kReportPeriodUs) {
        last_report_us_ = now;
        build_frame();
        last_byte_us_ = now;
        return;
    }

    // Shift out bytes at the line rate.
    while (frame_pos_ < kFrameLen && now - last_byte_us_ >= kByteTimeUs) {
        last_byte_us_ += kByteTimeUs;
        rx_.push(frame_[frame_pos_++]);
    }
}

u8 SimLaserUart::read(u8* dst, u8 len) {
    u8 n = 0;
    u8 b = 0;
    while (n < len && rx_.pop(&b)) dst[n++] = b;
    return n;
}

Status SimLaserUart::write(const u8* src, u8 len) {
    // The unit accepts configuration commands. Nothing in this project sends
    // any, so they are accepted and discarded rather than pretended about.
    (void)src;
    (void)len;
    return Status::kOk;
}

void SimCan::emit_heartbeats() {
    const u8 nodes[kNumNodes] = {kNodeDrive1, kNodeDrive2, kNodeGripper, kNodeSafety};
    for (u8 node : nodes) {
        if (!node_alive(node)) continue;
        CanFrame fr{};
        fr.id = static_cast<u16>(canid::kHeartbeatBase + node);
        fr.dlc = 1;
        fr.data[0] = 0x05;  // operational, in the CANopen sense
        rx_.push(fr);
    }
}

}  // namespace ppa
