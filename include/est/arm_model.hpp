// The firmware's own model of the arm.
//
// Deliberately separate from the plant. The plant knows the arm's true
// parameters; the firmware only knows the nominal drawing values from
// config.hpp. That gap is not an inconvenience to be minimised, it is the
// point: every real controller works from a model that is slightly wrong, and
// a project where the two agree exactly proves nothing.
//
// Float, no allocation, no host headers. This file cross compiles.
#pragma once

#include <cmath>

#include "config.hpp"
#include "est/lin.hpp"

namespace ppa {

constexpr float kGravityMS2 = 9.80665f;

// Link 1 nominal inertial parameters. Link 2's live in config.hpp because the
// payload gets folded into them at runtime.
constexpr float kLink1MassKg = 1.200f;
constexpr float kLink1ComM = 0.125f;

// Link inertias about each link's own centre of mass. Both links are treated
// as uniform rods, for which the inertia is m L^2 / 12:
//
//   link 1: 1.2 kg over 0.25 m  ->  0.00625 kg m^2
//   link 2: 0.8 kg over 0.20 m  ->  0.00267 kg m^2
constexpr float kLink1Inertia = 0.00625f;
constexpr float kLink2Inertia = 0.00267f;

// Raw contact force, with no filtering at all.
//
// WHY THIS EXISTS ALONGSIDE THE ESTIMATOR
//
// The Kalman filter gives an accurate contact force, and step 5 measured it
// at 0.1 N against 86.8 N true. But it takes about 33 ms to build that
// estimate after contact begins, because it has to accumulate evidence.
//
// That delay sets a speed limit on approaching a surface. At 50 mm/s, 33 ms
// is 1.65 mm of penetration, which against a 1.5e5 N/m contact is 248 N:
// measured, and over the mechanical limit.
//
// So detection and measurement are split. The motor current rises the instant
// the tool is blocked, with no filter in the way, because the velocity loop
// immediately demands more torque. Subtracting what the pose and the payload
// say the torque should be leaves the external part directly:
//
//   tau_ext = tau_motor - gravity - friction - inertia
//
// It is noisy, and during acceleration the inertia term makes it unreliable,
// which is why it is not used as the measurement. But while creeping toward
// a surface at a few millimetres a second the arm is almost stationary, the
// inertia term is negligible, and this is the fastest honest answer
// available.
//
// Fast detector, filtered magnitude. Each doing the job it is suited to.
struct RawContact {
    Vec<2> tau_ext_nm;
    float force_n;      // magnitude at the tool, via the Jacobian transpose
    // Signed vertical component, world frame, positive upward.
    //
    // The sign carries information the magnitude throws away. If the arm is
    // believed to be carrying half a kilogram and is not, it needs about
    // 2 Nm less torque than the model demands, which appears here as roughly
    // +5 N of upward force. That is a dropped load, visible in one control
    // cycle, and it needs no filtering: the only input is the torque the
    // drives report and the pose.
    //
    // It matters because the estimator cannot help here. The payload state is
    // deliberately frozen whenever the arm is in contact, so during a place
    // descent the filter is incapable of noticing that the box has gone.
    float force_y_n;
    bool valid;         // false near a singular pose
};

inline RawContact raw_contact(float q1, float q2, float qd1, float qd2,
                              float tau_motor1, float tau_motor2,
                              float payload_kg, float friction_scale);

// Drive constants, needed to turn a torque into a current and back.
//
//   tau_joint = N * eta * Kt * i
//
// Efficiency appears here because the gearbox eats some of it. Leaving eta
// out would make the firmware believe it has ten percent more torque than it
// does, and gravity feedforward would then undercompensate by ten percent.
struct DriveConstants {
    float gear_ratio;
    float torque_const;  // Nm per motor amp
    float efficiency;
    float winding_r;     // ohm
    float back_emf;      // V s per rad, motor side
    float current_limit;
    float bus_voltage;
    float rotor_inertia;   // kg m^2, motor side
    float visc_friction;   // Nm per rad/s, joint side
    float coulomb_friction;  // Nm, joint side
    // Joint speed at which back emf alone consumes the whole bus, with a
    // margin. Above this the drive has no voltage left to control with:
    //   omega_joint_max = v_bus / (Ke * N)
    // which is 9.6 rad/s for joint 1 and 16 rad/s for joint 2. Commanding
    // beyond it does not make the arm faster, it just saturates the loop.
    float speed_limit;
};

// Nominal drive data for the two joints, matching the parts fitted.
constexpr DriveConstants kDrive[2] = {
    {50.0f, 0.1f, 0.90f, 1.0f, 0.1f, 6.0f, 48.0f, 2.0e-5f, 0.05f, 0.15f, 8.0f},
    {30.0f, 0.1f, 0.90f, 1.0f, 0.1f, 6.0f, 48.0f, 2.0e-5f, 0.05f, 0.15f, 12.0f},
};

inline float torque_to_current(int joint, float torque_nm) {
    const DriveConstants& d = kDrive[joint];
    return torque_nm / (d.gear_ratio * d.efficiency * d.torque_const);
}

inline float current_to_torque(int joint, float current_a) {
    const DriveConstants& d = kDrive[joint];
    return current_a * d.gear_ratio * d.efficiency * d.torque_const;
}


// Link 2 inertial parameters with a payload of the given mass held at the
// tool. Same parallel axis arithmetic the plant uses, in float.
struct Link2 {
    float mass_kg;
    float com_m;
};

struct Link2Full {
    float mass_kg;
    float com_m;
    float inertia;
};

// NOTE ON NEGATIVE PAYLOAD, WHICH MATTERS MORE THAN IT SOUNDS
//
// These functions must stay smooth through payload_kg = 0, including for
// small negative values. A physically impossible negative payload is a
// perfectly ordinary thing for an estimator to hold briefly: the true value
// is zero, the noise is finite, so half the time the estimate is below zero.
//
// An earlier version returned the bare link whenever the payload was zero or
// negative. That made the function flat for all negative inputs, so the
// filter's numerical Jacobian came out as exactly zero, the Kalman gain for
// the payload state went to zero, and the state could never recover. It
// became an absorbing dead zone: once the estimate touched a negative value
// it stayed there forever with its initial variance intact.
//
// It was found because one box out of three failed to be weighed at all
// while the other two came out within four grams. The difference was only
// which side of zero the estimate happened to land on first.
//
// So: no early return. The formulas below are smooth, and the only guard is
// against a total mass at or below zero, which would be a different kind of
// wrong entirely.
inline Link2 link2_with_payload(float payload_kg) {
    const float m = kLink2MassKg + payload_kg;
    if (m <= 1.0e-3f) return Link2{kLink2MassKg, kLink2ComM};
    const float com = (kLink2MassKg * kLink2ComM + payload_kg * kLink2M) / m;
    return Link2{m, com};
}

// Same, including the inertia, by the parallel axis theorem. The payload is
// treated as a point mass at the tool.
//
// The inertia term is what makes a payload estimate observable during
// acceleration as well as at rest. Ignoring it would leave the filter able to
// see a payload only through gravity, which means only through pose, which
// means not at all while the arm holds still.
inline Link2Full link2_full(float payload_kg) {
    const float m = kLink2MassKg + payload_kg;
    if (m <= 1.0e-3f)
        return Link2Full{kLink2MassKg, kLink2ComM, kLink2Inertia};
    const float com = (kLink2MassKg * kLink2ComM + payload_kg * kLink2M) / m;
    const float d1 = com - kLink2ComM;
    const float d2 = kLink2M - com;
    const float inertia = kLink2Inertia + kLink2MassKg * d1 * d1 +
                          payload_kg * d2 * d2;
    return Link2Full{m, com, inertia};
}

// Gravity torque at each joint, for a given pose and payload.
//
//   G1 = (m1 lc1 + m2 l1) g cos(q1) + m2 lc2 g cos(q1 + q2)
//   G2 =  m2 lc2 g cos(q1 + q2)
//
// This is the single most useful thing the firmware can compute. Feeding it
// forward means the position loop no longer has to hold the arm up with
// integral action, which is what otherwise costs you either sag or overshoot.
//
// It is also where the payload estimate from step 5 plugs in: get the mass
// wrong and the arm sags by an amount proportional to the error, which is
// itself a signal.
inline Vec<2> gravity_torque(float q1, float q2, float payload_kg) {
    const Link2 l2 = link2_with_payload(payload_kg);
    const float c1 = std::cos(q1);
    const float c12 = std::cos(q1 + q2);

    Vec<2> g{};
    g[0] = (kLink1MassKg * kLink1ComM + l2.mass_kg * kLink1M) * kGravityMS2 * c1 +
           l2.mass_kg * l2.com_m * kGravityMS2 * c12;
    g[1] = l2.mass_kg * l2.com_m * kGravityMS2 * c12;
    return g;
}

// Inertia matrix M(q), including the rotor inertia each gearbox reflects to
// its joint. Only q2 appears, because the arm folds about joint 2.
//
//   M11 = I1 + m1 lc1^2 + I2 + m2 (l1^2 + lc2^2 + 2 l1 lc2 cos q2) + N1^2 Jm
//   M12 = M21 = I2 + m2 (lc2^2 + l1 lc2 cos q2)
//   M22 = I2 + m2 lc2^2 + N2^2 Jm
//
// The reflected rotor inertia is not a refinement. At a gear ratio of 50 it
// is N^2 Jm = 0.05 kg m^2, comparable to the whole arm, so leaving it out
// would make the model believe the arm is roughly half as heavy as it is.
inline Mat<2, 2> inertia_matrix(float q2, float payload_kg) {
    const Link2Full l2 = link2_full(payload_kg);
    const float c2 = std::cos(q2);

    Mat<2, 2> M{};
    M(0, 0) = kLink1Inertia + kLink1MassKg * kLink1ComM * kLink1ComM +
              l2.inertia +
              l2.mass_kg * (kLink1M * kLink1M + l2.com_m * l2.com_m +
                            2.0f * kLink1M * l2.com_m * c2) +
              kDrive[0].gear_ratio * kDrive[0].gear_ratio * kDrive[0].rotor_inertia;
    M(0, 1) = l2.inertia +
              l2.mass_kg * (l2.com_m * l2.com_m + kLink1M * l2.com_m * c2);
    M(1, 0) = M(0, 1);
    M(1, 1) = l2.inertia + l2.mass_kg * l2.com_m * l2.com_m +
              kDrive[1].gear_ratio * kDrive[1].gear_ratio * kDrive[1].rotor_inertia;
    return M;
}

// Coriolis and centrifugal torque, already multiplied by the joint rates.
//
//   C1 = -m2 l1 lc2 sin(q2) (2 qd1 qd2 + qd2^2)
//   C2 =  m2 l1 lc2 sin(q2) qd1^2
inline Vec<2> coriolis_torque(float q2, float qd1, float qd2, float payload_kg) {
    const Link2Full l2 = link2_full(payload_kg);
    const float h = l2.mass_kg * kLink1M * l2.com_m * std::sin(q2);
    Vec<2> c{};
    c[0] = -h * (2.0f * qd1 * qd2 + qd2 * qd2);
    c[1] = h * qd1 * qd1;
    return c;
}

// Joint friction, scaled by an estimated wear factor.
//
// The Coulomb term is smoothed with tanh so the filter's Jacobians stay
// finite through zero speed. A true signum here would make the derivative
// infinite at standstill and the covariance would blow up every time the arm
// stopped.
//
// This is the term the friction_scale state multiplies, and the reason it is
// separable from a payload: it reverses sign with the direction of motion,
// and gravity does not.
inline Vec<2> friction_torque(float qd1, float qd2, float scale) {
    constexpr float kEps = 0.01f;  // rad/s, matches the plant's smoothing
    Vec<2> f{};
    f[0] = scale * (kDrive[0].visc_friction * qd1 +
                    kDrive[0].coulomb_friction * std::tanh(qd1 / kEps));
    f[1] = scale * (kDrive[1].visc_friction * qd2 +
                    kDrive[1].coulomb_friction * std::tanh(qd2 / kEps));
    return f;
}

// Solve a 2x2 system in closed form. Returns false if singular, which for an
// inertia matrix should never happen and therefore means something upstream
// is wrong rather than something to paper over.
inline bool solve2x2(const Mat<2, 2>& A, const Vec<2>& b, Vec<2>* out) {
    const float det = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    if (std::fabs(det) < 1e-12f) return false;
    const float inv = 1.0f / det;
    (*out)[0] = inv * (A(1, 1) * b[0] - A(0, 1) * b[1]);
    (*out)[1] = inv * (A(0, 0) * b[1] - A(1, 0) * b[0]);
    return true;
}

// Tool position from joint angles.
inline Vec<2> forward_kinematics(float q1, float q2) {
    Vec<2> p{};
    p[0] = kLink1M * std::cos(q1) + kLink2M * std::cos(q1 + q2);
    p[1] = kLink1M * std::sin(q1) + kLink2M * std::sin(q1 + q2);
    return p;
}

// Inverse kinematics: joint angles that put the tool at a target.
//
// Two solutions exist inside the workspace, elbow up and elbow down. This
// arm reaches down and out to a table below its base, so elbow down is the
// one that keeps the elbow clear of the work; elbow up folds the arm back
// over itself and is reachable but useless here.
//
//   c2 = (x^2 + y^2 - l1^2 - l2^2) / (2 l1 l2)
//   q2 = atan2(-sqrt(1 - c2^2), c2)          elbow down takes the negative
//   q1 = atan2(y, x) - atan2(l2 sin q2, l1 + l2 cos q2)
//
// Returns false outside the annulus between |l1 - l2| and l1 + l2. A caller
// that ignores that gets a NaN pose, which the control loop will chase.
struct IkSolution {
    bool reachable;
    float q1;
    float q2;
};

inline IkSolution inverse_kinematics(float x, float y) {
    const float r2 = x * x + y * y;
    const float l1 = kLink1M;
    const float l2 = kLink2M;

    float c2 = (r2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    // Clamp before the sqrt. A target a micron outside the workspace would
    // otherwise produce a NaN rather than a boundary pose, and a NaN
    // setpoint propagates silently all the way to the motors.
    if (c2 > 1.0f) {
        if (c2 > 1.0f + 1.0e-4f) return IkSolution{false, 0.0f, 0.0f};
        c2 = 1.0f;
    }
    if (c2 < -1.0f) {
        if (c2 < -1.0f - 1.0e-4f) return IkSolution{false, 0.0f, 0.0f};
        c2 = -1.0f;
    }

    const float s2 = -std::sqrt(1.0f - c2 * c2);  // elbow down
    const float q2 = std::atan2(s2, c2);
    const float q1 = std::atan2(y, x) -
                     std::atan2(l2 * s2, l1 + l2 * c2);
    return IkSolution{true, wrap_pi(q1), wrap_pi(q2)};
}

// Jacobian, relating joint rates to tool velocity: v = J qd.
//
// Its transpose maps a tool force to joint torques, which is how step 5 will
// turn an estimated joint torque back into the contact force the firmware
// actually wants to know about.
inline Mat<2, 2> jacobian(float q1, float q2) {
    const float s1 = std::sin(q1);
    const float c1 = std::cos(q1);
    const float s12 = std::sin(q1 + q2);
    const float c12 = std::cos(q1 + q2);

    Mat<2, 2> J{};
    J(0, 0) = -kLink1M * s1 - kLink2M * s12;
    J(0, 1) = -kLink2M * s12;
    J(1, 0) = kLink1M * c1 + kLink2M * c12;
    J(1, 1) = kLink2M * c12;
    return J;
}

// Defined after the dynamics helpers it needs.
inline RawContact raw_contact(float q1, float q2, float qd1, float qd2,
                              float tau_motor1, float tau_motor2,
                              float payload_kg, float friction_scale) {
    RawContact out{};
    const Vec<2> G = gravity_torque(q1, q2, payload_kg);
    const Vec<2> Fr = friction_torque(qd1, qd2, friction_scale);
    const Vec<2> C = coriolis_torque(q2, qd1, qd2, payload_kg);

    // Inertia is deliberately left out. Including it would need the joint
    // accelerations, which are not measured, and differentiating the
    // reported velocities to get them would amplify their noise by the loop
    // rate. The cost is that this is only trustworthy while the arm is
    // nearly stationary, which is exactly when it is used.
    out.tau_ext_nm[0] = tau_motor1 - G[0] - Fr[0] - C[0];
    out.tau_ext_nm[1] = tau_motor2 - G[1] - Fr[1] - C[1];

    const Mat<2, 2> J = jacobian(q1, q2);
    const Mat<2, 2> Jt = transpose(J);
    const float det = Jt(0, 0) * Jt(1, 1) - Jt(0, 1) * Jt(1, 0);
    if (std::fabs(det) < 1.0e-4f) {
        out.force_n = 0.0f;
        out.force_y_n = 0.0f;
        out.valid = false;
        return out;
    }
    const float inv = 1.0f / det;
    const float fx = inv * (Jt(1, 1) * out.tau_ext_nm[0] -
                            Jt(0, 1) * out.tau_ext_nm[1]);
    const float fy = inv * (Jt(0, 0) * out.tau_ext_nm[1] -
                            Jt(1, 0) * out.tau_ext_nm[0]);
    out.force_n = std::sqrt(fx * fx + fy * fy);
    out.force_y_n = fy;
    out.valid = true;
    return out;
}

}  // namespace ppa
