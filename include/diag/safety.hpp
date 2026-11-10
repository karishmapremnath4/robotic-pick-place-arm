// Supervisory safety state machine.
//
// Detecting a fault is only half the job. Something then has to decide what
// the machine does about it, and that decision cannot live inside the
// estimator or the control loops: both of those are the things that might be
// wrong. So it sits above them, reads their health, and has the authority to
// take the arm's motion away.
//
// THREE STATES, AND WHY NOT MORE
//
//   kNormal     everything healthy. Full speed, full force envelope.
//   kDegraded   something is faulted but a healthy alternative exists. The
//               arm keeps working at reduced speed and a tighter force
//               limit, because a less certain estimate deserves less
//               authority.
//   kSafeStop   the arm cannot be trusted to move. Wheels, so to speak, off.
//
// A richer state machine is tempting and usually a mistake: every extra state
// is another transition to reason about and another path that will not be
// tested. Three states, with the trip reason recorded separately, carries the
// same information without the combinatorics.
//
// WHY SAFE STOP LATCHES
//
// Degraded clears itself when the condition clears. Safe stop does not: it
// holds until something outside the firmware acknowledges it.
//
// That asymmetry is deliberate. A fault that trips, clears, and trips again
// is the signature of an intermittent connection or a marginal part, and a
// machine that silently restarts each time will hide it until something
// breaks. Requiring an acknowledgement means every safe stop is seen by a
// person.
//
// THE COLLISION TRIP, AND WHY IT DOES NOT USE THE FORCE ESTIMATE
//
// The obvious design is to trip when the estimated contact force exceeds a
// threshold while the arm should be in free space. That does not work, and
// measuring it is what showed why: driving the arm into an obstacle at 68 N
// of real contact force produced an estimated force of 0.0 N.
//
// The reason is in the estimator itself. Free motion mode exists precisely
// because it asserts that nothing is touching the arm, and it enforces that
// by decaying the external torque toward zero. So in free motion the filter
// cannot report a contact force: it is built not to.
//
// But step 5 already measured where the force goes instead. It is absorbed
// into the payload, as a confident 6.940 kg phantom box.
//
// So that is the signal. The gripper's clamp force is 40 N against a jaw
// friction of 0.4, which with the acceleration allowance caps the mass it
// can physically hold at about 1.25 kg. A payload estimate above that is not
// a heavy box, it is something pushing on the arm. And a payload estimate of
// any size while the jaws are open is impossible for the same reason.
//
// That makes the trip a statement about physics rather than a threshold on a
// quantity the filter is not entitled to report.
#pragma once

#include "diag/sensor_health.hpp"
#include "types.hpp"

namespace ppa {

enum class SafetyState : u8 {
    kNormal = 0,
    kDegraded,
    kSafeStop,
};

enum class TripReason : u8 {
    kNone = 0,
    kDriveNodeLost,        // a motor drive stopped answering
    kPositionUnavailable,  // both position channels faulted
    kChannelDisagree,      // the two position channels contradict each other
    kUnexpectedContact,    // force appeared while nothing should be touching
    kOverForce,            // contact force beyond the mechanical limit
    kEstimatorDiverged,    // the filter has lost confidence entirely
    kDeadlineMiss,         // the control loop did not run on time
    kSensorFaulted,        // a sensor is out but the arm can continue
    kPayloadUntrusted,     // carrying something of unknown mass
};

struct SafetyLimits {
    // Any force at all in free motion means something is in the way. 15 N is
    // comfortably above the estimator's noise, which step 5 measured at well
    // under a newton, and far below anything that would damage the arm or
    // hurt a person leaning on it.
    float free_motion_force_n{15.0f};

    // The heaviest mass the gripper can physically hold: 40 N of clamp
    // against 0.4 jaw friction, divided by gravity plus the 3 m/s^2
    // acceleration allowance. A payload estimate above this is not a box.
    float max_grippable_kg{1.25f};

    // And with the jaws open, anything above the estimator's own noise floor
    // is impossible. Step 5 measured that floor at 17 g on an empty gripper.
    float open_jaw_payload_kg{0.15f};

    // The mechanical limit while deliberately in contact. Beyond this the
    // gearbox is the thing at risk, not the workpiece.
    float max_contact_force_n{120.0f};

    // The filter has stopped being useful if it is this unsure.
    //
    // Both of these MUST sit above the filter's own initial priors, which are
    // 0.5 kg for the payload and 25 Nm for the external torque on entering
    // contact. Set below them, a divergence trip fires on a filter that has
    // simply not converged yet, which is not the same thing at all: the first
    // version used 0.30 kg and the arm tripped to safe stop on its very first
    // control cycle, before it had seen a single measurement.
    //
    // So the test is "worse than knowing nothing", not "not yet certain".
    float max_payload_sigma_kg{0.80f};  // against a 0.5 kg prior
    float max_tau_sigma_nm{40.0f};      // against a 25 Nm contact prior

    // What degraded mode gives up.
    float degraded_speed_scale{0.5f};
    float degraded_force_n{40.0f};
};

// Everything the supervisor needs to see. Gathered by the caller rather than
// reached for, so the supervisor has no dependency on how any of it is
// obtained and can be tested on its own.
struct SafetyInputs {
    bool drive_node_lost[2]{false, false};
    bool gripper_node_lost{false};
    bool channels_disagree{false};
    float contact_force_n{0.0f};
    bool in_contact_mode{false};
    float payload_kg{0.0f};
    bool gripper_holding{false};
    // Whether the firmware asked for the jaws to be closed. A gripper that
    // was told to close and reports it is not holding has dropped something,
    // which is the task layer's problem. Only jaws that were deliberately
    // opened make a payload estimate physically impossible.
    bool grip_commanded_closed{false};
    float payload_sigma_kg{0.0f};
    float tau_sigma_nm{0.0f};
    bool payload_trustworthy{true};
    u32 deadline_misses{0};
};

class SafetySupervisor {
public:
    explicit SafetySupervisor(const SafetyLimits& l = SafetyLimits{}) : l_(l) {}

    void update(const SafetyInputs& in, const SensorHealth& health,
                TimeUs now_us);

    SafetyState state() const { return state_; }
    TripReason reason() const { return reason_; }

    // What the control layer is allowed to do. Reading these rather than
    // reading the state directly keeps the policy in one place.
    bool motion_allowed() const { return state_ != SafetyState::kSafeStop; }
    float speed_scale() const;
    float force_limit_n() const;

    // Clears a latched safe stop. Named for what it is: a person taking
    // responsibility, not a retry.
    void acknowledge();

    // Milliseconds from the condition appearing to the state changing.
    float trip_latency_ms() const { return trip_latency_ms_; }
    TimeUs tripped_at_us() const { return tripped_at_us_; }
    u32 trip_count() const { return trip_count_; }

    static const char* state_name(SafetyState s);
    static const char* reason_name(TripReason r);

private:
    void trip(SafetyState s, TripReason r, TimeUs now_us);

    SafetyLimits l_{};
    SafetyState state_{SafetyState::kNormal};
    TripReason reason_{TripReason::kNone};
    TimeUs tripped_at_us_{0};
    TimeUs condition_seen_us_{0};
    bool condition_pending_{false};
    float trip_latency_ms_{0.0f};
    u32 trip_count_{0};
};

}  // namespace ppa
