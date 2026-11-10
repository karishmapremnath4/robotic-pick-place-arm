// Two-link planar arm dynamics in a vertical plane.
//
// Standard manipulator equation:
//     M(q) * qdd + C(q, qd) * qd + G(q) = tau
//
// q1 is measured from the horizontal x axis. Gravity acts along -y, so the
// worst case gravity load is the arm straight out horizontal (q1 = q2 = 0).
#pragma once

#include <array>

namespace arm {

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<std::array<double, 2>, 2>;

constexpr double kGravity = 9.81;  // m/s^2

struct ArmParams {
    // Link 1
    double l1;   // length, m
    double lc1;  // distance from joint 1 to link 1 centre of mass, m
    double m1;   // mass, kg
    double I1;   // inertia about its own centre of mass, kg m^2

    // Link 2, with any payload already lumped into m2 / lc2 / I2.
    double l2;
    double lc2;
    double m2;
    double I2;
};

// Inertia matrix M(q). Only depends on q2, because the arm folds about joint 2.
Mat2 inertia(const ArmParams& p, const Vec2& q);

// Coriolis and centrifugal term, returned already multiplied by qd.
Vec2 coriolis(const ArmParams& p, const Vec2& q, const Vec2& qd);

// Gravity torque at each joint.
Vec2 gravity(const ArmParams& p, const Vec2& q);

// Forward kinematics: end effector position in metres.
Vec2 end_effector(const ArmParams& p, const Vec2& q);

// Solve M * qdd = rhs for qdd. Two by two, so closed form.
Vec2 solve2x2(const Mat2& M, const Vec2& rhs);

}  // namespace arm
