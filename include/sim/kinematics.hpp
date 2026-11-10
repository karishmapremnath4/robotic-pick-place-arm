// Forward and inverse kinematics for the two link planar arm.
//
// Forward: joint angles in, tip position out. Always has one answer.
// Inverse: tip position in, joint angles out. Has two answers inside the
// workspace (elbow up and elbow down), one on the boundary, and none outside.
#pragma once

#include "sim/arm_dynamics.hpp"

namespace arm {

enum class Elbow { kUp, kDown };

struct IkResult {
    bool reachable;
    Vec2 q;  // joint angles, rad
};

// Tip position for a given pose.
Vec2 forward_kinematics(const ArmParams& p, const Vec2& q);

// Joint angles that put the tip at target. Returns reachable = false if the
// target is outside the annulus between |l1 - l2| and l1 + l2.
IkResult inverse_kinematics(const ArmParams& p, const Vec2& target, Elbow elbow);

// Jacobian, relating joint rates to tip velocity: v = J * qd.
Mat2 jacobian(const ArmParams& p, const Vec2& q);

// Distance from a singular pose. Zero when the arm is straight or fully
// folded, where the Jacobian loses rank and the arm cannot move in every
// direction. Used as a guard on commanded paths.
double manipulability(const ArmParams& p, const Vec2& q);

}  // namespace arm
