// Brushed/BLDC motor with a reduction gearbox driving one joint.
//
// Electrical:   L * di/dt = V - R*i - Ke * w_motor
// Torque:       tau_motor = Kt * i
// Gearbox:      tau_joint = N * eta * tau_motor,   w_motor = N * w_joint
//
// The gearbox also reflects the rotor inertia to the joint as N^2 * Jm, which
// for this arm is a significant fraction of the link inertia and cannot be
// ignored.
#pragma once

namespace arm {

struct MotorParams {
    double R;    // winding resistance, ohm
    double L;    // winding inductance, H
    double Kt;   // torque constant, Nm/A
    double Ke;   // back emf constant, V s/rad (equal to Kt in SI)
    double Jm;   // rotor inertia, kg m^2
    double N;    // gear ratio, motor rev per joint rev
    double eta;  // gearbox efficiency, 0 to 1

    double i_cont;  // continuous current rating, A
    double i_peak;  // peak current rating, A
    double v_bus;   // dc bus voltage, V

    // Joint side friction.
    double b_visc;     // viscous, Nm per rad/s
    double f_coulomb;  // coulomb, Nm
};

// Inertia the gearbox reflects to the joint.
inline double reflected_inertia(const MotorParams& m) {
    return m.N * m.N * m.Jm;
}

// Joint torque produced by a given motor current.
inline double joint_torque(const MotorParams& m, double current) {
    return m.N * m.eta * m.Kt * current;
}

// Friction torque opposing joint motion. Coulomb term is smoothed with tanh so
// the integrator does not chatter around zero speed.
double friction_torque(const MotorParams& m, double joint_speed);

// di/dt for the winding, given terminal voltage and joint speed.
double current_derivative(const MotorParams& m, double current,
                          double joint_speed, double voltage);

}  // namespace arm
