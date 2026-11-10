#include "diag/safety.hpp"

#include <cmath>

namespace ppa {

const char* SafetySupervisor::state_name(SafetyState s) {
    switch (s) {
        case SafetyState::kNormal: return "NORMAL";
        case SafetyState::kDegraded: return "DEGRADED";
        case SafetyState::kSafeStop: return "SAFE STOP";
    }
    return "?";
}

const char* SafetySupervisor::reason_name(TripReason r) {
    switch (r) {
        case TripReason::kNone: return "none";
        case TripReason::kDriveNodeLost: return "drive node lost";
        case TripReason::kPositionUnavailable: return "no position channel left";
        case TripReason::kChannelDisagree: return "position channels disagree";
        case TripReason::kUnexpectedContact: return "unexpected contact";
        case TripReason::kOverForce: return "contact force over limit";
        case TripReason::kEstimatorDiverged: return "estimator diverged";
        case TripReason::kDeadlineMiss: return "control deadline missed";
        case TripReason::kSensorFaulted: return "sensor faulted";
        case TripReason::kPayloadUntrusted: return "payload mass unknown";
    }
    return "?";
}

void SafetySupervisor::trip(SafetyState s, TripReason r, TimeUs now_us) {
    // Never step back from safe stop inside this function. Only
    // acknowledge() leaves that state.
    if (state_ == SafetyState::kSafeStop && s != SafetyState::kSafeStop) return;

    if (s == state_ && r == reason_) return;

    if (s == SafetyState::kSafeStop && state_ != SafetyState::kSafeStop) {
        tripped_at_us_ = now_us;
        trip_latency_ms_ = condition_pending_
                               ? static_cast<float>(now_us - condition_seen_us_) * 1e-3f
                               : 0.0f;
        ++trip_count_;
    }
    state_ = s;
    reason_ = r;
}

void SafetySupervisor::update(const SafetyInputs& in, const SensorHealth& health,
                              TimeUs now_us) {
    // ---- conditions that stop the arm ------------------------------------
    //
    // Evaluated before the softer ones, and in rough order of how little
    // room for doubt each leaves.

    // A drive that has stopped answering is still powered and may still be
    // moving. The firmware has lost sight of it, which is worse than it
    // having stopped.
    if (in.drive_node_lost[0] || in.drive_node_lost[1]) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kDriveNodeLost, now_us);
        return;
    }

    // Both position channels gone. Nothing left to close a loop around.
    const bool enc_out = health.health(SensorId::kEncoders) == Health::kFaulted;
    const bool drv_out = health.health(SensorId::kDrivePosition) == Health::kFaulted;
    if (enc_out && drv_out) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kPositionUnavailable, now_us);
        return;
    }

    // Something pushing on the arm while nothing should be. Detected with no
    // force sensor and no bumper, and not through the force estimate, which
    // free motion mode is built not to produce. See the header.
    // Only act on a payload figure the filter says is worth acting on.
    //
    // Releasing a box reinflates the payload variance on purpose, because a
    // discontinuity is coming that the filter cannot predict. For a moment
    // afterwards the estimate still reads the old mass with a standard
    // deviation of 0.44 kg, which is the filter correctly saying it does not
    // know. Treating that as evidence of an external force tripped the arm
    // every single time it put a box down.
    //
    // So the rule is conditioned on trustworthiness. An estimate the filter
    // disowns is not evidence of anything.
    const bool impossible_payload =
        in.payload_trustworthy &&
        (in.payload_kg > l_.max_grippable_kg ||
         (!in.grip_commanded_closed && in.payload_kg > l_.open_jaw_payload_kg));

    if (!in.in_contact_mode && impossible_payload) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kUnexpectedContact, now_us);
        return;
    }

    // And the direct force test, which does apply once contact is declared.
    if (!in.in_contact_mode &&
        std::fabs(in.contact_force_n) > l_.free_motion_force_n) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kUnexpectedContact, now_us);
        return;
    }

    // Too much force even where contact was intended.
    if (std::fabs(in.contact_force_n) > l_.max_contact_force_n) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kOverForce, now_us);
        return;
    }

    // The filter has stopped knowing anything useful. Acting on its output
    // would be acting on noise.
    if (in.payload_sigma_kg > l_.max_payload_sigma_kg ||
        in.tau_sigma_nm > l_.max_tau_sigma_nm) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kEstimatorDiverged, now_us);
        return;
    }

    // A control loop that did not run on time means the arm was driven by a
    // stale command. On a 1 kHz loop that is a millisecond of open loop
    // motion, and it is not something to average over.
    if (in.deadline_misses > 0) {
        if (!condition_pending_) { condition_seen_us_ = now_us; condition_pending_ = true; }
        trip(SafetyState::kSafeStop, TripReason::kDeadlineMiss, now_us);
        return;
    }

    condition_pending_ = false;

    // A latched safe stop stays latched regardless of everything below.
    if (state_ == SafetyState::kSafeStop) return;

    // ---- conditions that reduce the arm's authority ----------------------

    // The two position channels contradict each other. One is wrong and two
    // channels cannot say which, so the arm continues cautiously while the
    // model based isolation works out the culprit.
    if (in.channels_disagree) {
        trip(SafetyState::kDegraded, TripReason::kChannelDisagree, now_us);
        return;
    }

    if (health.any_excluded() || in.gripper_node_lost) {
        trip(SafetyState::kDegraded, TripReason::kSensorFaulted, now_us);
        return;
    }

    // Carrying something whose mass is not known well enough to compensate
    // for. The arm can still move, but not quickly and not firmly.
    if (!in.payload_trustworthy) {
        trip(SafetyState::kDegraded, TripReason::kPayloadUntrusted, now_us);
        return;
    }

    trip(SafetyState::kNormal, TripReason::kNone, now_us);
}

float SafetySupervisor::speed_scale() const {
    switch (state_) {
        case SafetyState::kNormal: return 1.0f;
        case SafetyState::kDegraded: return l_.degraded_speed_scale;
        case SafetyState::kSafeStop: return 0.0f;
    }
    return 0.0f;
}

float SafetySupervisor::force_limit_n() const {
    switch (state_) {
        case SafetyState::kNormal: return l_.max_contact_force_n;
        case SafetyState::kDegraded: return l_.degraded_force_n;
        case SafetyState::kSafeStop: return 0.0f;
    }
    return 0.0f;
}

void SafetySupervisor::acknowledge() {
    if (state_ != SafetyState::kSafeStop) return;
    state_ = SafetyState::kNormal;
    reason_ = TripReason::kNone;
    condition_pending_ = false;
}

}  // namespace ppa
