// Machine configuration: numbers that are measured once when the cell is
// built, not derived and not tunable at runtime.
//
// This is firmware data, not simulation data. On a real machine it comes out
// of commissioning and gets loaded into flash. The simulation reads it from
// here rather than keeping its own copy, so the two can never drift apart:
// if this file is wrong, the arm is wrong in exactly the way a real
// mis-commissioned arm would be.
#pragma once

#include "types.hpp"

namespace ppa {

// Where each joint's encoder reads zero, relative to the joint's own zero.
//
// The encoders are single turn absolute, and these joints never travel a full
// revolution, so knowing this offset gives absolute joint position with no
// homing sequence at power on.
//
// Deliberately not round numbers. A real offset never is, and a firmware that
// silently assumed zero would pass a test written with zeros here.
constexpr float kEncoderZeroOffsetRad[2] = {0.3142f, -0.7854f};

// Link geometry the firmware needs for kinematics. Duplicated from the plant
// on purpose: the firmware is only entitled to the nominal drawing
// dimensions, not to the plant's true values. Keeping them separate is what
// lets a manufacturing tolerance be injected later as a fault.
constexpr float kLink1M = 0.250f;
constexpr float kLink2M = 0.200f;

// Where the laser ranger is mounted, relative to the tool point, and which
// way it looks.
//
// Offset sideways rather than on the tool centreline on purpose. A sensor
// looking straight down through the gripper is blinded by whatever the
// gripper is holding, so it can never measure the surface while carrying a
// box, which is exactly when you want to know how far above the surface you
// are. Mounting it beside the jaws costs a fixed offset and buys a reading
// that is always available.
constexpr float kLaserOffsetXM = 0.030f;

// Nominal link 2 inertial parameters, carrying nothing. The estimator needs
// these to predict how much torque the arm should require, and the difference
// between predicted and actual is where the payload estimate comes from.
constexpr float kLink2MassKg = 0.800f;
constexpr float kLink2ComM = 0.100f;

}  // namespace ppa
