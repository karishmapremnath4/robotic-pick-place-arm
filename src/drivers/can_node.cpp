#include "drivers/can_node.hpp"

namespace ppa {

namespace {

i16 get_le16(const u8* p) {
    return static_cast<i16>(static_cast<u16>(p[0]) |
                            (static_cast<u16>(p[1]) << 8));
}

void put_le16(u8* p, i16 v) {
    p[0] = static_cast<u8>(static_cast<u16>(v) & 0xFF);
    p[1] = static_cast<u8>((static_cast<u16>(v) >> 8) & 0xFF);
}

i16 sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return static_cast<i16>(v);
}

}  // namespace

int CanNodes::node_index(u8 node_id) {
    switch (node_id) {
        case kNodeDrive1: return 0;
        case kNodeDrive2: return 1;
        case kNodeGripper: return 2;
        case kNodeSafety: return 3;
        default: return -1;
    }
}

Status CanNodes::send_drive_current(float i1, float i2) {
    CanFrame fr{};
    fr.dlc = 2;

    fr.id = canid::kCmdDrive1;
    put_le16(&fr.data[0], sat16(i1 / kCurrentLsbA));
    Status st = bus_.send(fr);
    if (st != Status::kOk) { ++tx_errors_; return st; }

    fr.id = canid::kCmdDrive2;
    put_le16(&fr.data[0], sat16(i2 / kCurrentLsbA));
    st = bus_.send(fr);
    if (st != Status::kOk) { ++tx_errors_; return st; }
    return Status::kOk;
}

Status CanNodes::send_grip(bool close) {
    CanFrame fr{};
    fr.id = canid::kCmdGripper;
    fr.dlc = 1;
    fr.data[0] = close ? 1 : 0;
    const Status st = bus_.send(fr);
    if (st != Status::kOk) ++tx_errors_;
    return st;
}

void CanNodes::poll() {
    const TimeUs now = clk_.now_us();

    CanFrame fr{};
    while (bus_.receive(&fr)) {
        switch (fr.id) {
            case canid::kStatusDrive1:
            case canid::kStatusDrive2: {
                if (fr.dlc < 6) { ++unknown_frames_; break; }
                const int idx = (fr.id == canid::kStatusDrive1) ? 0 : 1;
                DriveStatus& d = drive_[idx];
                d.position_rad =
                    static_cast<float>(get_le16(&fr.data[0])) * kPosLsbRad;
                d.velocity_rad_s =
                    static_cast<float>(get_le16(&fr.data[2])) * kVelLsbRadS;
                d.current_a =
                    static_cast<float>(get_le16(&fr.data[4])) * kCurrentLsbA;
                d.stamp_us = now;
                d.valid = true;
                break;
            }
            case canid::kStatusGripper: {
                if (fr.dlc < 3) { ++unknown_frames_; break; }
                // State is an enumerated byte, so an unexpected value must be
                // rejected rather than cast blindly into the enum.
                if (fr.data[0] > static_cast<u8>(GripReport::kOpening)) {
                    ++unknown_frames_;
                    break;
                }
                grip_.state = static_cast<GripReport>(fr.data[0]);
                // Clamp force is unsigned: a gripper cannot pull.
                const u16 raw = static_cast<u16>(
                    static_cast<u16>(fr.data[1]) |
                    (static_cast<u16>(fr.data[2]) << 8));
                grip_.clamp_n = static_cast<float>(raw) * kClampLsbN;
                grip_.stamp_us = now;
                grip_.valid = true;
                break;
            }
            default: {
                if (fr.id > canid::kHeartbeatBase &&
                    fr.id <= canid::kHeartbeatBase + 0x7F) {
                    const u8 node = static_cast<u8>(fr.id - canid::kHeartbeatBase);
                    const int idx = node_index(node);
                    if (idx < 0) { ++unknown_frames_; break; }
                    last_heartbeat_us_[idx] = now;
                    seen_[idx] = true;
                    lost_latched_[idx] = false;
                } else {
                    ++unknown_frames_;
                }
                break;
            }
        }
    }

    for (int i = 0; i < kNumNodes; ++i) {
        if (!seen_[i]) continue;
        const bool stale = (now - last_heartbeat_us_[i]) > kHeartbeatTimeoutUs;
        if (stale && !lost_latched_[i]) {
            lost_latched_[i] = true;
            ++node_lost_events_;
        }
    }
}

bool CanNodes::node_lost(u8 node_id) const {
    const int idx = node_index(node_id);
    if (idx < 0) return false;
    if (!seen_[idx]) return false;
    return lost_latched_[idx];
}

}  // namespace ppa
