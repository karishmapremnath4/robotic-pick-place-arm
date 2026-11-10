#include "sim/motor.hpp"

#include <cmath>

namespace arm {

namespace {
// Speed below which the coulomb term is blended out, rad/s at the joint.
constexpr double kStribeckEps = 0.01;
}  // namespace

double friction_torque(const MotorParams& m, double joint_speed) {
    return m.b_visc * joint_speed
         + m.f_coulomb * std::tanh(joint_speed / kStribeckEps);
}

double current_derivative(const MotorParams& m, double current,
                          double joint_speed, double voltage) {
    const double w_motor = m.N * joint_speed;
    return (voltage - m.R * current - m.Ke * w_motor) / m.L;
}

}  // namespace arm
