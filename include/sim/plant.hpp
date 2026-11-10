// The full plant: two-link arm plus one motor and gearbox per joint.
//
// State is six numbers:  q1 q2 qd1 qd2 i1 i2
// Input is the two motor terminal voltages.
//
// Integrated with fixed step RK4. The step has to be small enough for the
// electrical dynamics, which are the fastest thing here: with L = 1 mH and
// R = 1 ohm the winding time constant is 1 ms, so a 10 us step gives 100
// points per time constant.
#pragma once

#include "sim/arm_dynamics.hpp"
#include "sim/motor.hpp"

#include <array>

namespace arm {

using State = std::array<double, 6>;

struct PlantParams {
    ArmParams arm;
    MotorParams motor[2];
};

class Plant {
public:
    explicit Plant(const PlantParams& p) : p_(p) {}

    // Advance one fixed step. Voltages are clamped to the dc bus.
    //
    // tau_ext is the external joint torque from anything touching the arm:
    // the table pushing back, a box being lifted, a collision. It enters the
    // manipulator equation exactly where a real contact force would, so the
    // firmware sees its effect only through motor current, which is the whole
    // point of the project.
    void step(const std::array<double, 2>& voltage, const Vec2& tau_ext,
              double dt);

    // The arm's own inertial parameters change when a payload is gripped or
    // released, so they are settable rather than fixed at construction.
    void set_arm(const ArmParams& a) { p_.arm = a; }

    // Joint friction rises with wear, so the motor parameters are settable
    // for the same reason. Both of these mutate parameters without touching
    // the integrator state, which is the point: rebuilding the plant to
    // change a parameter would silently reset the arm to its initial pose.
    void set_motor(int j, const MotorParams& m) { p_.motor[j] = m; }

    const State& state() const { return x_; }
    void set_state(const State& x) { x_ = x; }

    Vec2 q() const { return Vec2{x_[0], x_[1]}; }
    Vec2 qd() const { return Vec2{x_[2], x_[3]}; }
    Vec2 current() const { return Vec2{x_[4], x_[5]}; }

    Vec2 tip() const { return end_effector(p_.arm, q()); }

    const PlantParams& params() const { return p_; }

private:
    State derivative(const State& x, const std::array<double, 2>& voltage,
                     const Vec2& tau_ext) const;

    PlantParams p_;
    State x_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

// The draft specification. These numbers are a starting point, chosen so the
// gravity load at full extension is inside the continuous current rating.
// See README for the sizing calculation.
PlantParams default_params();

}  // namespace arm
