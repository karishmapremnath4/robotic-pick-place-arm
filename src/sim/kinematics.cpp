#include "sim/kinematics.hpp"

#include <cmath>

namespace arm {

Vec2 forward_kinematics(const ArmParams& p, const Vec2& q) {
    return end_effector(p, q);
}

IkResult inverse_kinematics(const ArmParams& p, const Vec2& target,
                            Elbow elbow) {
    const double x = target[0];
    const double y = target[1];
    const double r2 = x * x + y * y;
    const double r = std::sqrt(r2);

    const double r_max = p.l1 + p.l2;
    const double r_min = std::fabs(p.l1 - p.l2);
    if (r > r_max || r < r_min) {
        return IkResult{false, Vec2{0.0, 0.0}};
    }

    // Law of cosines on the triangle formed by the two links and the reach.
    double c2 = (r2 - p.l1 * p.l1 - p.l2 * p.l2) / (2.0 * p.l1 * p.l2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    const double q2 = (elbow == Elbow::kDown) ? std::acos(c2) : -std::acos(c2);
    const double q1 = std::atan2(y, x)
                    - std::atan2(p.l2 * std::sin(q2), p.l1 + p.l2 * std::cos(q2));

    return IkResult{true, Vec2{q1, q2}};
}

Mat2 jacobian(const ArmParams& p, const Vec2& q) {
    const double s1 = std::sin(q[0]);
    const double c1 = std::cos(q[0]);
    const double s12 = std::sin(q[0] + q[1]);
    const double c12 = std::cos(q[0] + q[1]);

    return Mat2{{{-p.l1 * s1 - p.l2 * s12, -p.l2 * s12},
                 { p.l1 * c1 + p.l2 * c12,  p.l2 * c12}}};
}

double manipulability(const ArmParams& p, const Vec2& q) {
    const Mat2 J = jacobian(p, q);
    // For a square Jacobian this reduces to the absolute determinant, which
    // works out to l1 * l2 * |sin(q2)|.
    return std::fabs(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
}

}  // namespace arm
