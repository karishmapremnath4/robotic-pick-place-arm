#include "est/ekf.hpp"

namespace ppa {

namespace {

// Step sizes for the numerical Jacobians. Each is small against the scale of
// its own state and large against float precision: too small and the
// difference is dominated by rounding, too large and it stops being a
// derivative. Angles and rates get 1e-4, torques 1e-3, the two scalars 1e-4.
constexpr float kEps[kNStates] = {
    1.0e-4f, 1.0e-4f,  // angles, rad
    1.0e-4f, 1.0e-4f,  // rates, rad/s
    1.0e-3f, 1.0e-3f,  // external torque, Nm
    1.0e-4f,           // payload, kg
    1.0e-4f,           // friction scale
};

}  // namespace

void Ekf::init(float q1, float q2, const EkfTuning& t) {
    t_ = t;
    x_ = StateVec{};
    x_[kStQ1] = q1;
    x_[kStQ2] = q2;
    // Friction starts at its nominal value, not at zero. Starting at zero
    // would tell the filter the joints are frictionless, which is a worse
    // guess than the drawing value and takes real time to unlearn.
    x_[kStFriction] = 1.0f;

    p_ = StateMat{};
    p_(kStQ1, kStQ1) = t_.p0_angle * t_.p0_angle;
    p_(kStQ2, kStQ2) = t_.p0_angle * t_.p0_angle;
    p_(kStQd1, kStQd1) = t_.p0_rate * t_.p0_rate;
    p_(kStQd2, kStQd2) = t_.p0_rate * t_.p0_rate;
    p_(kStTauExt1, kStTauExt1) = t_.p0_tau_ext * t_.p0_tau_ext;
    p_(kStTauExt2, kStTauExt2) = t_.p0_tau_ext * t_.p0_tau_ext;
    p_(kStPayload, kStPayload) = t_.p0_payload * t_.p0_payload;
    p_(kStFriction, kStFriction) = t_.p0_friction * t_.p0_friction;
    rejected_ = 0;
}

void Ekf::set_mode(EkfMode m) {
    if (m == mode_) return;
    if (m == EkfMode::kContact) {
        // About to touch something, so the external torque could be
        // anything. Say so rather than letting the filter resist the
        // evidence for the first half second of contact.
        const float v = t_.p_tau_ext_contact * t_.p_tau_ext_contact;
        p_(kStTauExt1, kStTauExt1) = v;
        p_(kStTauExt2, kStTauExt2) = v;
    }
    mode_ = m;
}

StateVec Ekf::dynamics(const StateVec& x, const Vec<2>& tau_motor,
                       float dt) const {
    const float q1 = x[kStQ1];
    const float q2 = x[kStQ2];
    const float qd1 = x[kStQd1];
    const float qd2 = x[kStQd2];
    const float payload = x[kStPayload];
    const float fscale = x[kStFriction];

    const Mat<2, 2> M = inertia_matrix(q2, payload);
    const Vec<2> C = coriolis_torque(q2, qd1, qd2, payload);
    const Vec<2> G = gravity_torque(q1, q2, payload);
    const Vec<2> F = friction_torque(qd1, qd2, fscale);

    // The manipulator equation, rearranged for acceleration:
    //   M qdd = tau_motor + tau_ext - C - G - friction
    Vec<2> rhs{};
    rhs[0] = tau_motor[0] + x[kStTauExt1] - C[0] - G[0] - F[0];
    rhs[1] = tau_motor[1] + x[kStTauExt2] - C[1] - G[1] - F[1];

    Vec<2> qdd{};
    if (!solve2x2(M, rhs, &qdd)) {
        qdd[0] = 0.0f;
        qdd[1] = 0.0f;
    }

    StateVec out = x;
    out[kStQ1] = wrap_pi(q1 + qd1 * dt);
    out[kStQ2] = wrap_pi(q2 + qd2 * dt);
    out[kStQd1] = qd1 + qdd[0] * dt;
    out[kStQd2] = qd2 + qdd[1] * dt;

    // In free motion the external torque decays toward zero, because nothing
    // is touching the arm. In contact it holds its value and the process
    // noise lets it move.
    if (mode_ == EkfMode::kFreeMotion) {
        const float decay = 1.0f - dt / t_.tau_ext_decay_s;
        const float k = decay > 0.0f ? decay : 0.0f;
        out[kStTauExt1] = x[kStTauExt1] * k;
        out[kStTauExt2] = x[kStTauExt2] * k;
    }
    // The four inferred states are modelled as random walks: they hold their
    // value through the prediction and their uncertainty grows. That is the
    // honest model. A payload does not evolve according to any law the
    // firmware knows, it just changes when the gripper acts.
    return out;
}

void Ekf::predict(const Vec<2>& motor_torque_nm, float dt) {
    last_motor_torque_ = motor_torque_nm;

    // Numerical state transition Jacobian, by central differences.
    StateMat F{};
    for (int j = 0; j < kNStates; ++j) {
        StateVec xp = x_;
        StateVec xm = x_;
        xp[j] += kEps[j];
        xm[j] -= kEps[j];
        const StateVec fp = dynamics(xp, motor_torque_nm, dt);
        const StateVec fm = dynamics(xm, motor_torque_nm, dt);
        const float inv = 1.0f / (2.0f * kEps[j]);
        for (int i = 0; i < kNStates; ++i) {
            // Angle rows must difference through the wrap, or a state sitting
            // near pi produces a derivative of about 2 pi over 2 eps.
            float d = fp[i] - fm[i];
            if (i == kStQ1 || i == kStQ2) d = wrap_pi(d);
            F(i, j) = d * inv;
        }
    }

    x_ = dynamics(x_, motor_torque_nm, dt);

    // Process noise. Variances scale with dt so the tuning is independent of
    // the filter rate.
    StateMat Q{};
    Q(kStQ1, kStQ1) = t_.q_angle * t_.q_angle * dt;
    Q(kStQ2, kStQ2) = t_.q_angle * t_.q_angle * dt;
    Q(kStQd1, kStQd1) = t_.q_rate * t_.q_rate * dt;
    Q(kStQd2, kStQd2) = t_.q_rate * t_.q_rate * dt;
    // The mode picks which set of process noises applies. In free motion the
    // external torque is held down so the payload and friction have to
    // explain the residual; in contact it is released and they are frozen.
    const bool contact = (mode_ == EkfMode::kContact);
    const float q_tau = contact ? t_.q_tau_ext_contact : t_.q_tau_ext_free;
    const float q_pay = contact ? t_.q_payload_contact : t_.q_payload_free;
    const float q_fric = contact ? t_.q_friction_contact : t_.q_friction_free;

    Q(kStTauExt1, kStTauExt1) = q_tau * q_tau * dt;
    Q(kStTauExt2, kStTauExt2) = q_tau * q_tau * dt;
    Q(kStPayload, kStPayload) = q_pay * q_pay * dt;
    Q(kStFriction, kStFriction) = q_fric * q_fric * dt;

    p_ = symmetrise(F * p_ * transpose(F) + Q);
}

Vec<2> Ekf::predict_tool_accel(const StateVec& x) const {
    const float q1 = x[kStQ1];
    const float q2 = x[kStQ2];
    const float qd1 = x[kStQd1];
    const float qd2 = x[kStQd2];

    // Joint accelerations implied by the current state and the known input.
    const Mat<2, 2> M = inertia_matrix(q2, x[kStPayload]);
    const Vec<2> C = coriolis_torque(q2, qd1, qd2, x[kStPayload]);
    const Vec<2> G = gravity_torque(q1, q2, x[kStPayload]);
    const Vec<2> Fr = friction_torque(qd1, qd2, x[kStFriction]);

    Vec<2> rhs{};
    rhs[0] = last_motor_torque_[0] + x[kStTauExt1] - C[0] - G[0] - Fr[0];
    rhs[1] = last_motor_torque_[1] + x[kStTauExt2] - C[1] - G[1] - Fr[1];

    Vec<2> qdd{};
    if (!solve2x2(M, rhs, &qdd)) {
        qdd[0] = 0.0f;
        qdd[1] = 0.0f;
    }

    // Tool acceleration: a = J qdd + Jdot qd.
    const float s1 = std::sin(q1);
    const float c1 = std::cos(q1);
    const float s12 = std::sin(q1 + q2);
    const float c12 = std::cos(q1 + q2);
    const float w12 = qd1 + qd2;

    const float J00 = -kLink1M * s1 - kLink2M * s12;
    const float J01 = -kLink2M * s12;
    const float J10 = kLink1M * c1 + kLink2M * c12;
    const float J11 = kLink2M * c12;

    const float D00 = -kLink1M * c1 * qd1 - kLink2M * c12 * w12;
    const float D01 = -kLink2M * c12 * w12;
    const float D10 = -kLink1M * s1 * qd1 - kLink2M * s12 * w12;
    const float D11 = -kLink2M * s12 * w12;

    const float ax = J00 * qdd[0] + J01 * qdd[1] + D00 * qd1 + D01 * qd2;
    const float ay = J10 * qdd[0] + J11 * qdd[1] + D10 * qd1 + D11 * qd2;

    // Specific force: acceleration minus gravity, then rotated into the tool
    // frame, which is turned by q1 + q2.
    const float fx = ax;
    const float fy = ay + kGravityMS2;
    Vec<2> out{};
    out[0] = fx * c12 + fy * s12;
    out[1] = -fx * s12 + fy * c12;
    return out;
}

template <int M>
float Ekf::apply_update(const Mat<M, kNStates>& H, const Vec<M>& innovation,
                        const Mat<M, M>& R, bool fuse) {
    const Mat<kNStates, M> Ht = transpose(H);
    const Mat<M, M> S = H * p_ * Ht + R;

    Mat<M, M> Sinv{};
    if (!invert(S, &Sinv)) {
        // A singular innovation covariance means the update is meaningless.
        // Rejecting it is the only safe choice: pushing on would put NaNs
        // through the whole state and there would be no way back.
        ++rejected_;
        return 1.0e9f;
    }

    // Normalised innovation squared, the quantity step 6 gates on.
    float nis = 0.0f;
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j) nis += innovation[i] * Sinv(i, j) * innovation[j];

    // Monitor only: the caller wants the disagreement, not its effect.
    if (!fuse) return nis;

    // Single sample outlier gate. Rejecting here keeps one wild reading from
    // kicking the state before the fault logic has seen enough samples to
    // judge the sensor.
    if (t_.gate_chi2 > 0.0f && nis > t_.gate_chi2) {
        ++gated_;
        return nis;
    }

    const Mat<kNStates, M> K = p_ * Ht * Sinv;

    for (int i = 0; i < kNStates; ++i) {
        float d = 0.0f;
        for (int j = 0; j < M; ++j) d += K(i, j) * innovation[j];
        x_[i] += d;
    }
    x_[kStQ1] = wrap_pi(x_[kStQ1]);
    x_[kStQ2] = wrap_pi(x_[kStQ2]);

    // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T.
    //
    // The short form P = (I - K H) P is algebraically identical and
    // numerically worse: it subtracts two similar quantities and can drive
    // the covariance indefinite after enough updates, at which point the
    // filter quietly stops working. Joseph form costs two more matrix
    // products and is a sum of two positive terms, so it cannot.
    const StateMat I = StateMat::identity();
    const StateMat A = I - K * H;
    p_ = symmetrise(A * p_ * transpose(A) + K * R * transpose(K));
    return nis;
}

float Ekf::update_encoders(float q1, float q2, bool fuse) {
    Mat<2, kNStates> H{};
    H(0, kStQ1) = 1.0f;
    H(1, kStQ2) = 1.0f;

    Vec<2> y{};
    y[0] = wrap_pi(q1 - x_[kStQ1]);
    y[1] = wrap_pi(q2 - x_[kStQ2]);

    Mat<2, 2> R{};
    R(0, 0) = t_.r_encoder * t_.r_encoder;
    R(1, 1) = t_.r_encoder * t_.r_encoder;
    return apply_update(H, y, R, fuse);
}

float Ekf::update_drive_position(float q1, float q2, bool fuse) {
    Mat<2, kNStates> H{};
    H(0, kStQ1) = 1.0f;
    H(1, kStQ2) = 1.0f;

    Vec<2> y{};
    y[0] = wrap_pi(q1 - x_[kStQ1]);
    y[1] = wrap_pi(q2 - x_[kStQ2]);

    // Inflate with speed to cover the frame's unknown age on the bus.
    const float s1 = t_.r_drive_pos +
                     t_.r_drive_pos_latency_s * std::fabs(x_[kStQd1]);
    const float s2 = t_.r_drive_pos +
                     t_.r_drive_pos_latency_s * std::fabs(x_[kStQd2]);

    Mat<2, 2> R{};
    R(0, 0) = s1 * s1;
    R(1, 1) = s2 * s2;
    return apply_update(H, y, R, fuse);
}

float Ekf::update_drive_velocity(float qd1, float qd2, bool fuse) {
    Mat<2, kNStates> H{};
    H(0, kStQd1) = 1.0f;
    H(1, kStQd2) = 1.0f;

    Vec<2> y{};
    y[0] = qd1 - x_[kStQd1];
    y[1] = qd2 - x_[kStQd2];

    // Inflate with the joint accelerations, for the same reason the position
    // channel is inflated with speed: the frame is up to a millisecond old.
    const Mat<2, 2> M = inertia_matrix(x_[kStQ2], x_[kStPayload]);
    const Vec<2> C = coriolis_torque(x_[kStQ2], x_[kStQd1], x_[kStQd2], x_[kStPayload]);
    const Vec<2> G = gravity_torque(x_[kStQ1], x_[kStQ2], x_[kStPayload]);
    const Vec<2> Fr = friction_torque(x_[kStQd1], x_[kStQd2], x_[kStFriction]);
    Vec<2> rhs{};
    rhs[0] = last_motor_torque_[0] + x_[kStTauExt1] - C[0] - G[0] - Fr[0];
    rhs[1] = last_motor_torque_[1] + x_[kStTauExt2] - C[1] - G[1] - Fr[1];
    Vec<2> qdd{};
    if (!solve2x2(M, rhs, &qdd)) { qdd[0] = 0.0f; qdd[1] = 0.0f; }

    const float sv1 = t_.r_drive_vel +
                      t_.r_drive_vel_latency_s * std::fabs(qdd[0]);
    const float sv2 = t_.r_drive_vel +
                      t_.r_drive_vel_latency_s * std::fabs(qdd[1]);

    Mat<2, 2> R{};
    R(0, 0) = sv1 * sv1;
    R(1, 1) = sv2 * sv2;
    return apply_update(H, y, R, fuse);
}

float Ekf::update_gyro(float tool_rate, bool fuse) {
    // The tool gyro sees the sum of both joint rates, so this one
    // measurement constrains a combination rather than either state alone.
    Mat<1, kNStates> H{};
    H(0, kStQd1) = 1.0f;
    H(0, kStQd2) = 1.0f;

    Vec<1> y{};
    y[0] = tool_rate - (x_[kStQd1] + x_[kStQd2]);

    Mat<1, 1> R{};
    R(0, 0) = t_.r_gyro * t_.r_gyro;
    return apply_update(H, y, R, fuse);
}

float Ekf::update_tool_accel(float ax, float ay, bool fuse) {
    // This is the measurement that carries the force information. The
    // accelerometer sees acceleration directly, and acceleration is where an
    // external torque shows up immediately, rather than after being
    // integrated into a velocity change.
    const Vec<2> h = predict_tool_accel(x_);

    Mat<2, kNStates> H{};
    for (int j = 0; j < kNStates; ++j) {
        StateVec xp = x_;
        StateVec xm = x_;
        xp[j] += kEps[j];
        xm[j] -= kEps[j];
        const Vec<2> hp = predict_tool_accel(xp);
        const Vec<2> hm = predict_tool_accel(xm);
        const float inv = 1.0f / (2.0f * kEps[j]);
        H(0, j) = (hp[0] - hm[0]) * inv;
        H(1, j) = (hp[1] - hm[1]) * inv;
    }

    Vec<2> y{};
    y[0] = ax - h[0];
    y[1] = ay - h[1];

    // Acceleration dependent measurement noise. The joint accelerations the
    // prediction rests on are where its error lives, so the harder the arm
    // is accelerating the less this measurement deserves to be believed.
    const Mat<2, 2> M = inertia_matrix(x_[kStQ2], x_[kStPayload]);
    const Vec<2> C = coriolis_torque(x_[kStQ2], x_[kStQd1], x_[kStQd2], x_[kStPayload]);
    const Vec<2> G = gravity_torque(x_[kStQ1], x_[kStQ2], x_[kStPayload]);
    const Vec<2> Fr = friction_torque(x_[kStQd1], x_[kStQd2], x_[kStFriction]);
    Vec<2> rhs{};
    rhs[0] = last_motor_torque_[0] + x_[kStTauExt1] - C[0] - G[0] - Fr[0];
    rhs[1] = last_motor_torque_[1] + x_[kStTauExt2] - C[1] - G[1] - Fr[1];
    Vec<2> qdd{};
    float qdd_mag = 0.0f;
    if (solve2x2(M, rhs, &qdd))
        qdd_mag = std::sqrt(qdd[0] * qdd[0] + qdd[1] * qdd[1]);

    const float sigma = t_.r_accel + t_.r_accel_per_qdd * qdd_mag;

    Mat<2, 2> R{};
    R(0, 0) = sigma * sigma;
    R(1, 1) = sigma * sigma;
    return apply_update(H, y, R, fuse);
}

void Ekf::announce_grip_change() {
    // Reinflate the payload variance to its initial value. The filter is
    // about to be shown evidence of a discontinuity it cannot predict, and a
    // confident wrong prior would take seconds to overcome.
    p_(kStPayload, kStPayload) = t_.p0_payload * t_.p0_payload;
}

bool Ekf::contact_force(Vec<2>* force_n) const {
    const Mat<2, 2> J = jacobian(x_[kStQ1], x_[kStQ2]);
    const Mat<2, 2> Jt = transpose(J);

    // Near a singular pose the arm cannot sense force along its own length,
    // so this inverse is genuinely ill conditioned rather than merely
    // awkward. Report the failure instead of a large number.
    const float det = Jt(0, 0) * Jt(1, 1) - Jt(0, 1) * Jt(1, 0);
    if (std::fabs(det) < 1.0e-4f) return false;

    Mat<2, 2> Jinv{};
    if (!invert(Jt, &Jinv)) return false;

    const Vec<2> tau = tau_ext();
    (*force_n)[0] = Jinv(0, 0) * tau[0] + Jinv(0, 1) * tau[1];
    (*force_n)[1] = Jinv(1, 0) * tau[0] + Jinv(1, 1) * tau[1];
    return true;
}

float Ekf::payload_sigma() const {
    const float v = p_(kStPayload, kStPayload);
    return v > 0.0f ? std::sqrt(v) : 0.0f;
}

float Ekf::tau_ext_sigma(int joint) const {
    const int i = (joint == 0) ? kStTauExt1 : kStTauExt2;
    const float v = p_(i, i);
    return v > 0.0f ? std::sqrt(v) : 0.0f;
}

bool Ekf::payload_trustworthy() const {
    // A payload estimate is only worth acting on once the filter is confident
    // to better than about 50 grams. Below that confidence the number exists
    // but means little, and the grip verification in step 7 must not decide
    // anything from it.
    return payload_sigma() < 0.05f;
}

}  // namespace ppa
