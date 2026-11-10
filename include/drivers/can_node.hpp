// CAN bus driver for the PPA-100 cell.
//
// Four nodes: two servo drives, the gripper, and a safety relay.
//
// The drives report their own position and velocity from their own internal
// encoders. The controller also has its own encoders on SPI. That is not
// waste. Two independent position channels is how safety rated motion is
// built, and two channels that can disagree is precisely what makes the
// fault detection possible: a stuck SPI encoder is invisible on its own, and
// obvious the moment it is compared against the drive.
//
// Physical values are scaled to integers the way a real device sends them, so
// the driver has to undo the scaling and the sign extension. Range choices
// and why they fit are noted next to each scale factor.
//
// Nothing here blocks, and the receive queue is always drained to empty. A
// partly drained queue grows a backlog, and then every value the firmware
// acts on is older than it looks.
#pragma once

#include "hal/bus.hpp"
#include "rtos/ring.hpp"

namespace ppa {

namespace canid {
// Host to node.
constexpr u16 kCmdDrive1 = 0x201;
constexpr u16 kCmdDrive2 = 0x202;
constexpr u16 kCmdGripper = 0x203;
// Node to host.
constexpr u16 kStatusDrive1 = 0x181;
constexpr u16 kStatusDrive2 = 0x182;
constexpr u16 kStatusGripper = 0x183;
constexpr u16 kHeartbeatBase = 0x700;  // plus node id
}  // namespace canid

constexpr u8 kNodeDrive1 = 1;
constexpr u8 kNodeDrive2 = 2;
constexpr u8 kNodeGripper = 3;
constexpr u8 kNodeSafety = 4;
constexpr int kNumNodes = 4;

// Three missed heartbeats at 10 Hz. Tight enough that a dead drive is caught
// well inside a move, loose enough that one late frame is not a fault.
constexpr TimeUs kHeartbeatTimeoutUs = 300000;

// Scale factors, matching what the nodes transmit.
//   position  0.001 rad/LSB, so +-32.767 rad covers the +-pi joint range
//   velocity  0.001 rad/s per LSB, +-32.7 rad/s against a 20 rad/s limit
//   current   1 mA/LSB, +-32.7 A against a 6 A peak rating
//   command   1 mA/LSB, the drive's torque request, same scale as feedback
//   clamp     0.1 N/LSB, +-3276 N against a 40 N gripper
constexpr float kPosLsbRad = 0.001f;
constexpr float kVelLsbRadS = 0.001f;
constexpr float kCurrentLsbA = 0.001f;

constexpr float kClampLsbN = 0.1f;

struct DriveStatus {
    float position_rad{0.0f};
    float velocity_rad_s{0.0f};
    float current_a{0.0f};
    TimeUs stamp_us{0};
    bool valid{false};
};

// Gripper states as the node reports them, matching the mechanism's own
// state machine rather than the firmware's idea of it.
enum class GripReport : u8 { kOpen = 0, kClosing = 1, kHolding = 2, kOpening = 3 };

struct GripperStatus {
    GripReport state{GripReport::kOpen};
    float clamp_n{0.0f};
    TimeUs stamp_us{0};
    bool valid{false};
};

class CanNodes {
public:
    CanNodes(CanBus& bus, const Clock& clk) : bus_(bus), clk_(clk) {}

    // Drains the receive queue and ages the heartbeats. Must be called at
    // least as often as the fastest frame on the bus, which is the 1 kHz
    // drive status.
    void poll();

    // The drives take a CURRENT setpoint, not a voltage. That is not a
    // style choice: a 10 kHz current loop needs 300 us of CAN per cycle
    // against a 100 us budget, so current has to be closed inside the drive.
    // See current_loop.hpp for the arithmetic.
    Status send_drive_current(float i1, float i2);
    Status send_grip(bool close);

    const DriveStatus& drive(int i) const { return drive_[i & 1]; }
    const GripperStatus& gripper() const { return grip_; }

    // True once a node has been silent past the timeout. Latches until the
    // node speaks again. A node never seen at all is not reported lost,
    // because at startup it may simply not have spoken yet.
    bool node_lost(u8 node_id) const;
    u32 node_lost_count() const { return node_lost_events_; }

    u32 unknown_frames() const { return unknown_frames_; }
    u32 tx_errors() const { return tx_errors_; }

private:
    static int node_index(u8 node_id);

    CanBus& bus_;
    const Clock& clk_;

    DriveStatus drive_[2]{};
    GripperStatus grip_{};
    TimeUs last_heartbeat_us_[kNumNodes]{};
    bool seen_[kNumNodes]{};
    bool lost_latched_[kNumNodes]{};
    u32 node_lost_events_{0};
    u32 unknown_frames_{0};
    u32 tx_errors_{0};
};

}  // namespace ppa
