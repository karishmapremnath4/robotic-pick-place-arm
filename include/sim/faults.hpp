// Fault injection knobs, set live from the console.
//
// Each one corresponds to something that really goes wrong on a robot cell,
// and each is injected at the lowest possible level: at the device, the
// sensor, or the mechanics. Never inside the estimator. The firmware has no
// idea a fault was injected, so if it notices, it noticed the way it would on
// a real machine.
#pragma once

#include "types.hpp"

namespace ppa {

struct Faults {
    // --- I2C, end effector inertial sensor -------------------------------
    bool imu_stuck{false};       // keeps returning its last conversion
    bool imu_nack{false};        // stops acknowledging its address
    float imu_noise_mult{1.0f};  // degraded part or a loose mount

    // --- SPI, joint encoders ---------------------------------------------
    bool enc_error_frames{false};  // device raises its error flag
    int enc_stuck_joint{0};        // 1 or 2, angle freezes. 0 for none

    // --- CAN, motor drives and safety node -------------------------------
    bool can_bus_off{false};
    u8 can_silent_node{0};

    // --- current sensing --------------------------------------------------
    // A drifting current sensor is the nastiest fault in this machine. The
    // contact force estimate is built from current, so a bias here becomes a
    // phantom force the arm will try to control against.
    float current_bias_a[2]{0.0f, 0.0f};

    // --- mechanics --------------------------------------------------------
    // Friction rising with wear. This is the deep one: extra friction and a
    // genuine contact force look identical to a torque residual estimator.
    // Telling them apart is the interesting problem in this project.
    float friction_mult{1.0f};

    bool grip_leak{false};        // clamp force bleeds away, the box slips
    bool obstacle_present{false}; // something is in the path that should not be

    // --- UART, laser distance sensor --------------------------------------
    bool laser_dropouts{false};   // intermittent missing returns
    bool laser_silent{false};     // stopped transmitting entirely

    void clear() { *this = Faults{}; }
};

}  // namespace ppa
