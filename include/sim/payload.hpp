// Combining a gripped payload into the arm's inertial parameters.
//
// This is the hinge the whole project turns on. When the gripper closes on a
// box, link 2 becomes heavier, its centre of mass shifts outward, and its
// inertia grows. The firmware is never told any of this. It only sees that
// the motors are working harder than the model predicts, and the estimator
// has to conclude "I am carrying about half a kilogram" from that alone.
//
// So the plant must represent the change honestly, including the inertia
// term. Getting the centre of mass shift right but not the inertia would
// make the payload easier to estimate than it really is.
#pragma once

#include "sim/arm_dynamics.hpp"

namespace arm {

// bare is link 2 carrying nothing. The payload is treated as a point mass at
// distance r from joint 2, which for a box held at the tool is the link
// length.
//
//   m'   = m + mp
//   lc'  = (m*lc + mp*r) / m'
//   I'   = I + m*(lc' - lc)^2 + mp*(r - lc')^2
//
// The two squared terms are the parallel axis theorem: both the link and the
// payload are now rotating about a centre of mass that is not their own.
inline ArmParams with_payload(const ArmParams& bare, double payload_kg,
                              double payload_r_m) {
    if (payload_kg <= 0.0) return bare;

    ArmParams p = bare;
    const double m = bare.m2;
    const double lc = bare.lc2;
    const double mp = payload_kg;
    const double r = payload_r_m;

    p.m2 = m + mp;
    p.lc2 = (m * lc + mp * r) / p.m2;
    p.I2 = bare.I2 + m * (p.lc2 - lc) * (p.lc2 - lc) +
           mp * (r - p.lc2) * (r - p.lc2);
    return p;
}

}  // namespace arm
