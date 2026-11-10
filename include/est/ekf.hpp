// Extended Kalman filter for the arm.
//
// WHAT IT ESTIMATES, AND WHY THOSE THINGS
//
//   0,1  q1, q2            joint angles          measured, two channels
//   2,3  qd1, qd2          joint rates           measured by the drives
//   4,5  tau_ext1, tau_ext2  external joint torque   NOT MEASURABLE
//   6    payload             mass in the gripper     NOT MEASURABLE
//   7    friction_scale      joint wear factor       NOT MEASURABLE
//
// The first four are already measured well, so the filter mostly smooths and
// cross checks them. The last four are the point. There is no force sensor and
// no scale on this machine, so all of them have to be inferred from the gap
// between what the model says the arm should be doing and what the sensors
// say it is doing.
//
// WHY FRICTION NEEDS A STATE OF ITS OWN
//
// A payload and a worn joint both make the motors pull more current, so one
// current reading cannot separate them. What separates them is direction:
// friction always opposes motion, so its contribution flips when the arm
// reverses, while gravity does not care which way the arm is moving.
//
// Giving friction its own state lets the filter use that. Without it, wear
// would be absorbed into the payload estimate and the arm would think it was
// carrying a box that is not there. That was measured in step 1: tripling the
// joint friction moves the direction dependent part of the current by 59 mA
// and the direction independent part by 6 mA, while a 0.5 kg payload does the
// opposite, 318 mA and 12 mA.
//
// WHAT IS AND IS NOT OBSERVABLE, CORRECTED
//
// The obvious worry is that a payload and a constant external torque are not
// separately observable at a fixed pose, since both are simply a constant
// torque. That worry is correct for a filter that treats the external torque
// as a free random walk, and an earlier version of this comment claimed the
// filter was therefore only trustworthy on a moving arm.
//
// Measurement showed otherwise, and the reason is the decay below. In free
// motion the external torque is pulled toward zero, which is a statement that
// nothing is touching the arm. That assumption is the missing constraint:
// with the external torque required to be near zero, a persistent torque has
// only one thing left to explain it, and the payload comes out to within a
// few grams even with the arm standing still.
//
// So the real limitation is not standing still. It is the assumption itself.
// If the arm IS touching something while the filter is in free motion mode,
// the contact force is misattributed to a phantom payload, and the filter
// will be confident about it. est_check demonstrates exactly that, because it
// is the failure the task layer in step 7 has to avoid and the reason the
// mode is set explicitly rather than guessed.
//
// WHY THERE ARE TWO MODES, AND WHY ONE WOULD NOT DO
//
// External torque, payload and friction all present themselves the same way:
// the motors are working harder than the model predicts. A single filter
// cannot identify all three at once, and the first version of this file
// proved it by estimating a 19.9 kg payload for a 0.25 kg box.
//
// The reason is the process noise on the external torque. Contact appears in
// milliseconds and reaches tens of newton metres, so catching it needs a
// large process noise. But a large process noise lets the external torque
// absorb the whole residual within one step, leaving nothing for the payload
// or the friction to explain. The two requirements are directly opposed:
//
//   weighing a box       wants external torque pinned near zero
//   catching a collision wants external torque free to move fast
//
// So the filter runs in one of two modes, and the task state machine says
// which:
//
//   kFreeMotion   the arm is moving in open space. No contact is expected,
//                 so the external torque is held near zero and the payload
//                 and friction are identified against the residual.
//
//   kContact      the arm is touching something. The external torque is
//                 released, and the payload and friction are frozen at
//                 whatever was identified in free motion.
//
// This is not a workaround. It is what industrial arms do: a payload
// identification move on pick up, then the identified value held fixed while
// the tool is in contact. Stating the mode explicitly also means the firmware
// can never quietly trust a payload figure it had no way to measure.
//
// JACOBIANS
//
// Computed numerically, by central differences on the nonlinear model. The
// analytical alternative needs the derivatives of the inertia matrix, the
// Coriolis term and the gravity vector with respect to eight states, which is
// a great deal of algebra to get subtly wrong. Numerical differencing is
// correct by construction and its cost is bounded and measurable: seventeen
// dynamics evaluations per prediction. est_check reports the measured figure.
#pragma once

#include "est/arm_model.hpp"
#include "est/lin.hpp"

namespace ppa {

constexpr int kNStates = 8;

enum StateIndex : int {
    kStQ1 = 0,
    kStQ2 = 1,
    kStQd1 = 2,
    kStQd2 = 3,
    kStTauExt1 = 4,
    kStTauExt2 = 5,
    kStPayload = 6,
    kStFriction = 7,
};

using StateVec = Vec<kNStates>;
using StateMat = Mat<kNStates, kNStates>;

// Which regime the arm is in. Set by the task layer, never inferred here: the
// firmware knows whether it commanded an approach, and guessing from the
// residual would be circular.
enum class EkfMode : u8 {
    kFreeMotion,
    kContact,
};

struct EkfTuning {
    // Process noise, as standard deviations per square root of a second. The
    // sqrt(dt) scaling keeps these independent of the filter rate, which
    // matters because the rate is a tuning knob.
    // Process noise on the joint angles.
    //
    // Set to 1e-4 originally, on the reasoning that q integrates qd exactly
    // so the relation contributes no noise of its own. The reasoning is
    // right and the conclusion was wrong: the relation is exact but the rate
    // it integrates is not, and the resulting position error is correlated
    // over many steps, which a single step covariance propagation cannot
    // represent. One millisecond of propagated rate uncertainty came out at
    // 60 microradians while the measured transient lag was 5 milliradians,
    // a hundredfold under.
    //
    // The consequence was a healthy encoder reaching a NIS of 97 during
    // ordinary step moves, which would have had the fault logic declaring
    // the best sensor on the arm broken every time it moved quickly.
    //
    // 0.12 rad per root second gives about 4 mrad of angle uncertainty per
    // millisecond step, which covers the observed lag. This is process noise
    // doing the job it exists for: absorbing model error that is real but
    // not worth modelling explicitly.
    float q_angle{0.12f};
    float q_rate{2.0f};        // unmodelled dynamics, in rad/s^2
    // External torque, in the two modes. Several orders of magnitude apart,
    // which is the whole point.
    float q_tau_ext_free{0.05f};     // no contact expected, so hold it down
    float q_tau_ext_contact{40.0f};  // contact arrives in milliseconds

    // In free motion the external torque is not merely held still, it is
    // pulled back toward zero with this time constant.
    //
    // A random walk that merely stays put is not enough. It starts with
    // whatever initial uncertainty it was given, and two free torque states
    // can then absorb a payload's entire gravity signature without ever
    // moving. That is what happened on the second attempt at this filter:
    // the contact force came out right and the payload gained no information
    // at all, its variance sitting exactly where it started.
    //
    // Decaying it encodes something the firmware actually knows: if nothing
    // is touching the arm, the external torque is zero.
    float tau_ext_decay_s{0.10f};

    // Payload and friction are identified in free motion and frozen in
    // contact, because in contact they are not separable from the contact
    // force itself.
    float q_payload_free{0.02f};   // kg/s
    float q_payload_contact{1.0e-4f};
    float q_friction_free{0.05f};  // per second
    float q_friction_contact{1.0e-4f};

    // Measurement noise, as standard deviations. Deliberately a little larger
    // than the values the device models actually use: on real hardware the
    // true noise is never known, and tuning to the exact figure is a way of
    // fooling yourself.
    // A little larger than the device models actually use, but only a
    // little. The first version inflated these three to six times the real
    // noise, which drove the mean normalised innovation squared down to 0.25
    // when it should sit near the measurement dimension. An overstated R
    // makes the filter ignore its sensors.
    float r_encoder{0.0008f};    // rad, devices give about 0.0006
    float r_drive_pos{0.0006f};  // rad, devices give 0.0004

    // Extra drive position noise per rad/s of joint speed.
    //
    // The drives report position over CAN at 1 kHz, so a reading can be up
    // to a millisecond old by the time the filter sees it, while the SPI
    // encoder is read fresh in the same cycle. At 5 rad/s that is 5 mrad of
    // completely genuine disagreement between two healthy sensors, and the
    // filter cannot satisfy both: its NIS reached 113 on a healthy arm
    // during ordinary moves.
    //
    // The clean fix is to timestamp the frame and compensate. The cheap and
    // honest fix is to admit the staleness as uncertainty, which is what
    // this does. 1.5 ms covers the 1 ms period plus jitter.
    float r_drive_pos_latency_s{0.0015f};
    float r_drive_vel{0.008f};   // rad/s, devices give 0.010

    // Extra drive velocity noise per rad/s^2 of joint acceleration. Same
    // staleness argument as the position channel, one derivative up: a
    // velocity report up to a millisecond old differs from the present
    // velocity by the acceleration times that millisecond. Measured worst
    // NIS on a healthy arm was 909 without this.
    float r_drive_vel_latency_s{0.0020f};
    float r_gyro{0.006f};        // rad/s, devices give 0.004
    // Set from the measured per sensor NIS, not from the device noise. The
    // accelerometer's prediction depends on the joint accelerations and so on
    // every state, which makes its model error much larger than its sensor
    // noise. At 0.15 its NIS came out at 8.1 against an expected 2, meaning
    // the filter was trusting it about twice as far as it deserved.
    float r_accel{0.30f};        // m/s^2, at a standstill

    // Extra accelerometer noise per rad/s^2 of joint acceleration.
    //
    // The accelerometer's prediction runs the whole manipulator equation to
    // get joint accelerations, then maps them through the Jacobian. So its
    // model error does not sit still: it scales with how hard the arm is
    // accelerating. Measured on a healthy arm, its NIS reached 50,644 during
    // large step moves against an expected 2, which makes a fixed R
    // meaningless and any fault threshold built on it useless.
    //
    // Inflating R with acceleration is the standard remedy and it is
    // physically honest: the measurement really is less informative when the
    // model it is compared against is being pushed hardest.
    float r_accel_per_qdd{0.20f};  // m/s^2 per rad/s^2

    // Initial uncertainty. The angles are known from the encoders at startup;
    // the four inferred states are not known at all.
    float p0_angle{1.0e-3f};
    float p0_rate{1.0e-2f};
    // Initial external torque uncertainty in free motion. Small, because in
    // free motion there is nothing touching the arm. Entering contact mode
    // reinflates it to the figure below.
    float p0_tau_ext{0.3f};          // Nm
    float p_tau_ext_contact{25.0f};  // Nm, on entering contact
    float p0_payload{0.5f};    // kg
    float p0_friction{1.0f};   // dimensionless scale, so 3x wear is reachable

    // Per sample innovation gate, as a chi squared value. A single update
    // this far outside its own predicted spread is an outlier, and applying
    // it would kick the state before any fault logic has had a chance to
    // notice. Set to zero to disable.
    //
    // This is distinct from the fault declaration in diag/sensor_health.hpp,
    // and both are needed. This one rejects one bad sample. That one decides
    // a sensor is broken after a run of them.
    // Per sample innovation gate, disabled by default.
    //
    // Measurement killed this idea. On a healthy arm the encoder NIS reaches
    // 92 and the drive position NIS 111 during ordinary step moves, because
    // those sensors are so precise that their S is tiny and the model's
    // transient lag dominates. Any gate tight enough to catch an outlier
    // also rejects good data at exactly the moment the filter needs it, and
    // at a gate of 25 the position estimate degraded enough that the arm
    // failed to complete a pick.
    //
    // Sustained faults are caught by the run length logic in
    // diag/sensor_health.hpp instead, which is where that job belongs.
    float gate_chi2{0.0f};
};

class Ekf {
public:
    void init(float q1, float q2, const EkfTuning& t = EkfTuning{});

    // Set by the task layer before each phase of a move.
    // Changing mode is a statement of knowledge, not just a tuning switch,
    // so entering contact reinflates the external torque uncertainty: the
    // firmware has just admitted it no longer knows that nothing is pushing
    // on the arm.
    void set_mode(EkfMode m);
    EkfMode mode() const { return mode_; }

    // Propagate to the next step. motor_torque is the joint torque the drives
    // are producing, computed from measured current, and is treated as a
    // known input rather than as a measurement.
    void predict(const Vec<2>& motor_torque_nm, float dt);

    // One update per sensor, kept separate on purpose. Step 6 has to be able
    // to reject one sensor without losing the others, and a single stacked
    // update would make that impossible.
    //
    // Each returns the normalised innovation squared, which is the quantity
    // the fault gating in step 6 tests against a chi squared threshold. A
    // return of a large number means the sensor disagreed with the filter by
    // far more than their combined uncertainty allows.
    //
    // fuse = false computes the innovation and its NIS but leaves the state
    // and covariance alone. That is not a debugging convenience, it is how a
    // faulted sensor earns its way back: its disagreement is measured
    // against a filter it is no longer influencing, which is a far stronger
    // test than one it is helping to shape. A sensor that is quietly wrong
    // can look consistent while it is being fused, because it is dragging
    // the estimate toward itself.
    float update_encoders(float q1, float q2, bool fuse = true);
    float update_drive_position(float q1, float q2, bool fuse = true);
    float update_drive_velocity(float qd1, float qd2, bool fuse = true);
    float update_gyro(float tool_rate, bool fuse = true);
    float update_tool_accel(float ax, float ay, bool fuse = true);

    // Called when the gripper opens or closes. The payload is about to change
    // discontinuously, and a filter that has spent ten seconds becoming
    // confident there is no box will fight the evidence that one has just
    // been picked up. Widening the payload variance lets it believe the new
    // measurements quickly. This is standard practice and it is honest: the
    // firmware genuinely does know a discontinuity is coming.
    void announce_grip_change();

    const StateVec& state() const { return x_; }
    const StateMat& covariance() const { return p_; }

    float q1() const { return x_[kStQ1]; }
    float q2() const { return x_[kStQ2]; }
    float qd1() const { return x_[kStQd1]; }
    float qd2() const { return x_[kStQd2]; }
    // The raw state, which can be slightly negative when the true payload is
    // zero. That is normal and the model is smooth through it.
    float payload_raw_kg() const { return x_[kStPayload]; }

    // What callers should act on: never negative, because no gripper has ever
    // held a negative mass. Clamping the output rather than the state is
    // deliberate; clamping the state would reintroduce the dead zone.
    float payload_kg() const {
        const float p = x_[kStPayload];
        return p > 0.0f ? p : 0.0f;
    }
    float friction_scale() const { return x_[kStFriction]; }

    Vec<2> tau_ext() const {
        Vec<2> t{};
        t[0] = x_[kStTauExt1];
        t[1] = x_[kStTauExt2];
        return t;
    }

    // The contact force at the tool, in world coordinates.
    //
    // tau = J^T F, so F = (J^T)^-1 tau. Near a singular pose J loses rank and
    // this inverse blows up, which is physically real: a fully extended arm
    // genuinely cannot sense a force along its own length. Returns false
    // there rather than returning a huge number.
    bool contact_force(Vec<2>* force_n) const;

    float payload_sigma() const;
    float tau_ext_sigma(int joint) const;
    bool payload_trustworthy() const;

    u32 rejected_updates() const { return rejected_; }
    u32 gated_updates() const { return gated_; }

private:
    // The nonlinear process model, shared by the propagation and by the
    // numerical Jacobian so the two can never disagree.
    StateVec dynamics(const StateVec& x, const Vec<2>& tau_motor, float dt) const;

    // The tool accelerometer's prediction, which needs the joint
    // accelerations and therefore the whole dynamics.
    Vec<2> predict_tool_accel(const StateVec& x) const;

    // Shared update step. Joseph form, then forced symmetry.
    template <int M>
    float apply_update(const Mat<M, kNStates>& H, const Vec<M>& innovation,
                       const Mat<M, M>& R, bool fuse);

    StateVec x_{};
    StateMat p_{};
    EkfTuning t_{};
    Vec<2> last_motor_torque_{};
    EkfMode mode_{EkfMode::kFreeMotion};
    u32 rejected_{0};
    u32 gated_{0};
};

}  // namespace ppa
