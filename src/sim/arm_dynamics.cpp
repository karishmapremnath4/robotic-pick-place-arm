#include "sim/arm_dynamics.hpp"

#include <cmath>

namespace arm {

Mat2 inertia(const ArmParams& p, const Vec2& q) {
    const double c2 = std::cos(q[1]);

    const double d11 = p.m1 * p.lc1 * p.lc1
                     + p.m2 * (p.l1 * p.l1 + p.lc2 * p.lc2 + 2.0 * p.l1 * p.lc2 * c2)
                     + p.I1 + p.I2;
    const double d12 = p.m2 * (p.lc2 * p.lc2 + p.l1 * p.lc2 * c2) + p.I2;
    const double d22 = p.m2 * p.lc2 * p.lc2 + p.I2;

    return Mat2{{{d11, d12}, {d12, d22}}};
}

Vec2 coriolis(const ArmParams& p, const Vec2& q, const Vec2& qd) {
    // h collects the pose dependent coupling term. It is zero when the arm is
    // straight or fully folded, and largest at q2 = +/- 90 degrees.
    const double h = -p.m2 * p.l1 * p.lc2 * std::sin(q[1]);

    const double c1 = h * qd[1] * qd[1] + 2.0 * h * qd[0] * qd[1];
    const double c2 = -h * qd[0] * qd[0];
    return Vec2{c1, c2};
}

Vec2 gravity(const ArmParams& p, const Vec2& q) {
    const double g1 = (p.m1 * p.lc1 + p.m2 * p.l1) * kGravity * std::cos(q[0])
                    + p.m2 * p.lc2 * kGravity * std::cos(q[0] + q[1]);
    const double g2 = p.m2 * p.lc2 * kGravity * std::cos(q[0] + q[1]);
    return Vec2{g1, g2};
}

Vec2 end_effector(const ArmParams& p, const Vec2& q) {
    const double x = p.l1 * std::cos(q[0]) + p.l2 * std::cos(q[0] + q[1]);
    const double y = p.l1 * std::sin(q[0]) + p.l2 * std::sin(q[0] + q[1]);
    return Vec2{x, y};
}

Vec2 solve2x2(const Mat2& M, const Vec2& rhs) {
    const double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    const double inv_det = 1.0 / det;
    return Vec2{(M[1][1] * rhs[0] - M[0][1] * rhs[1]) * inv_det,
                (-M[1][0] * rhs[0] + M[0][0] * rhs[1]) * inv_det};
}

}  // namespace arm
