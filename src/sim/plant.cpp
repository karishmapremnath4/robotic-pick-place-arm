#include "sim/plant.hpp"

#include <algorithm>

namespace arm {

State Plant::derivative(const State& x, const std::array<double, 2>& v,
                        const Vec2& tau_ext) const {
    const Vec2 q{x[0], x[1]};
    const Vec2 qd{x[2], x[3]};
    const Vec2 i{x[4], x[5]};

    // Mechanical side. The reflected rotor inertia adds to the diagonal of the
    // arm inertia matrix, because the rotor has to be accelerated too.
    Mat2 M = inertia(p_.arm, q);
    M[0][0] += reflected_inertia(p_.motor[0]);
    M[1][1] += reflected_inertia(p_.motor[1]);

    const Vec2 C = coriolis(p_.arm, q, qd);
    const Vec2 G = gravity(p_.arm, q);

    Vec2 rhs{};
    for (int j = 0; j < 2; ++j) {
        const double tau_drive = joint_torque(p_.motor[j], i[j]);
        const double tau_fric = friction_torque(p_.motor[j], qd[j]);
        rhs[j] = tau_drive + tau_ext[j] - C[j] - G[j] - tau_fric;
    }
    const Vec2 qdd = solve2x2(M, rhs);

    // Electrical side.
    const double di0 = current_derivative(p_.motor[0], i[0], qd[0], v[0]);
    const double di1 = current_derivative(p_.motor[1], i[1], qd[1], v[1]);

    return State{qd[0], qd[1], qdd[0], qdd[1], di0, di1};
}

void Plant::step(const std::array<double, 2>& voltage, const Vec2& tau_ext,
                 double dt) {
    std::array<double, 2> v = voltage;
    for (int j = 0; j < 2; ++j) {
        v[j] = std::clamp(v[j], -p_.motor[j].v_bus, p_.motor[j].v_bus);
    }

    auto add = [](const State& a, const State& b, double s) {
        State r{};
        for (std::size_t k = 0; k < a.size(); ++k) r[k] = a[k] + s * b[k];
        return r;
    };

    // The contact torque is held constant across the four stages. It is
    // recomputed by the world once per step, outside the integrator, because
    // resolving contact inside each RK4 stage would need the world state to
    // be re-evaluated at each stage too. At a 10 us step the contact dynamics
    // are heavily oversampled, so the error from freezing it is negligible.
    const State k1 = derivative(x_, v, tau_ext);
    const State k2 = derivative(add(x_, k1, 0.5 * dt), v, tau_ext);
    const State k3 = derivative(add(x_, k2, 0.5 * dt), v, tau_ext);
    const State k4 = derivative(add(x_, k3, dt), v, tau_ext);

    for (std::size_t k = 0; k < x_.size(); ++k) {
        x_[k] += (dt / 6.0) * (k1[k] + 2.0 * k2[k] + 2.0 * k3[k] + k4[k]);
    }

    // Drive current limit.
    //
    // Without this the only thing bounding current is the bus voltage across
    // the winding resistance, which for a 48 V bus and a 1 ohm winding is
    // 48 A: eight times the 6 A peak rating, and enough torque to fling the
    // arm hard enough to saturate a 4 g accelerometer. Every real drive
    // limits current, both to protect the transistors and to protect the
    // gearbox, so the plant has to as well or it flatters the controller with
    // authority the hardware does not have.
    //
    // Clamping the state after integration is a crude model of a current
    // limit: it is what a hard limiter does, but it says nothing about how
    // the limiter behaves at the boundary. A proper current loop with its own
    // bandwidth and anti windup is step 4's job. Until then this at least
    // keeps the arm inside its ratings.
    for (int j = 0; j < 2; ++j) {
        const double lim = p_.motor[j].i_peak;
        if (x_[4 + j] > lim) x_[4 + j] = lim;
        if (x_[4 + j] < -lim) x_[4 + j] = -lim;
    }
}

PlantParams default_params() {
    PlantParams p{};

    // The BARE arm, carrying nothing. A gripped box is added at runtime by
    // with_payload(), because the whole project rests on the arm's inertial
    // parameters changing when it picks something up. Baking a payload in
    // here would hide the very effect being estimated.
    //
    // Link 2 alone: 0.8 kg, centre of mass 0.10 m out, and for a uniform rod
    // of length 0.20 m the inertia about its own centre is m L^2 / 12, which
    // is 0.8 * 0.04 / 12 = 0.00267 kg m^2.
    p.arm = ArmParams{
        /*l1=*/0.25, /*lc1=*/0.125, /*m1=*/1.2, /*I1=*/0.00625,
        /*l2=*/0.20, /*lc2=*/0.100, /*m2=*/0.8, /*I2=*/0.00267};

    MotorParams base{};
    base.R = 1.0;
    base.L = 1.0e-3;
    base.Kt = 0.1;
    base.Ke = 0.1;
    base.Jm = 2.0e-5;
    base.eta = 0.90;
    base.i_cont = 2.0;
    base.i_peak = 6.0;
    base.v_bus = 48.0;
    base.b_visc = 0.05;
    base.f_coulomb = 0.15;

    p.motor[0] = base;
    p.motor[0].N = 50.0;  // joint 1 holds the whole arm against gravity

    p.motor[1] = base;
    p.motor[1].N = 30.0;  // joint 2 only holds link 2 and the payload

    return p;
}

}  // namespace arm
