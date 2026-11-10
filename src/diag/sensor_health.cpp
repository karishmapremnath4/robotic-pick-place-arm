#include "diag/sensor_health.hpp"

#include <cmath>

namespace ppa {

const char* SensorHealth::name(SensorId s) {
    switch (s) {
        case SensorId::kEncoders: return "spi encoders";
        case SensorId::kDrivePosition: return "drive position";
        case SensorId::kDriveVelocity: return "drive velocity";
        case SensorId::kToolImu: return "tool imu";
        case SensorId::kLaser: return "laser ranger";
        default: return "?";
    }
}

const char* SensorHealth::health_name(Health h) {
    switch (h) {
        case Health::kOk: return "ok";
        case Health::kSuspect: return "suspect";
        case Health::kFaulted: return "FAULTED";
        case Health::kProbation: return "probation";
    }
    return "?";
}

void SensorHealth::declare_fault(SensorId s, TimeUs now_us) {
    SensorStatus& st = status_[static_cast<int>(s)];
    if (st.health == Health::kFaulted) return;
    st.health = Health::kFaulted;
    st.faulted_at_us = now_us;
    ++st.fault_events;
    st.consecutive_passes = 0;
}

void SensorHealth::report_nis(SensorId s, float nis, int dof, TimeUs now_us) {
    SensorStatus& st = status_[static_cast<int>(s)];

    const float gate = kChiSq999[dof > 0 && dof < 4 ? dof : 2];
    const bool failed = !(nis < gate);  // written so a NaN counts as a failure

    if (failed) {
        if (st.consecutive_fails == 0) st.first_fail_us = now_us;
        ++st.consecutive_fails;
        st.consecutive_passes = 0;
        ++st.total_rejected;

        // A sensor on probation has already been proven broken once. One
        // more failure is enough to send it straight back, without waiting
        // for a fresh run of fifteen. Probation must not be a one way door:
        // the first version left a sensor lingering in probation
        // indefinitely, excluded from the filter but invisible to the
        // supervisor, so the arm reported itself NORMAL while running on a
        // single position channel.
        if (st.health == Health::kProbation) {
            declare_fault(s, now_us);
            return;
        }
        if (st.health == Health::kOk) st.health = Health::kSuspect;
        if (st.consecutive_fails >= p_.fault_after) declare_fault(s, now_us);
        return;
    }

    ++st.consecutive_passes;
    st.consecutive_fails = 0;

    switch (st.health) {
        case Health::kSuspect:
            // A short run of failures that stopped. Nothing is wrong; the
            // gate is probabilistic and this is what the tail looks like.
            st.health = Health::kOk;
            break;
        case Health::kFaulted:
            st.health = Health::kProbation;
            st.consecutive_passes = 1;
            break;
        case Health::kProbation:
            if (st.consecutive_passes >= p_.recover_after) st.health = Health::kOk;
            break;
        case Health::kOk:
            break;
    }
}

void SensorHealth::report_hard_fault(SensorId s, TimeUs now_us) {
    SensorStatus& st = status_[static_cast<int>(s)];
    if (st.consecutive_fails == 0 && st.health != Health::kFaulted)
        st.first_fail_us = now_us;
    st.consecutive_fails = p_.fault_after;
    declare_fault(s, now_us);
}

void SensorHealth::clear_hard_fault(SensorId s) {
    SensorStatus& st = status_[static_cast<int>(s)];
    if (st.health == Health::kFaulted) {
        st.health = Health::kProbation;
        st.consecutive_passes = 0;
        st.consecutive_fails = 0;
    }
}

void SensorHealth::report_channel_pair(float encoder_q1, float encoder_q2,
                                       float drive_q1, float drive_q2,
                                       float est_qd1, float est_qd2, float dt_s,
                                       TimeUs now_us) {
    // --- arbitration in progress -----------------------------------------
    if (arbitrating_) {
        arb_pred_delta_[0] += est_qd1 * dt_s;
        arb_pred_delta_[1] += est_qd2 * dt_s;
        ++arb_samples_;

        if (arb_samples_ >= p_.arbitrate_samples) {
            // How far each channel says it went, against how far the rates
            // say it went.
            const float enc_err =
                std::fabs((encoder_q1 - arb_enc_start_[0]) - arb_pred_delta_[0]) +
                std::fabs((encoder_q2 - arb_enc_start_[1]) - arb_pred_delta_[1]);
            const float drv_err =
                std::fabs((drive_q1 - arb_drv_start_[0]) - arb_pred_delta_[0]) +
                std::fabs((drive_q2 - arb_drv_start_[1]) - arb_pred_delta_[1]);

            const SensorId guilty = (enc_err > drv_err) ? SensorId::kEncoders
                                                        : SensorId::kDrivePosition;
            declare_fault(guilty, now_us);
            status_[static_cast<int>(guilty)].first_fail_us = arb_started_us_;
            verdict_ = guilty;
            have_verdict_ = true;
            arbitrating_ = false;
            channels_disagree_ = false;
            disagree_run_ = 0;
        }
        return;
    }

    const float d1 = std::fabs(encoder_q1 - drive_q1);
    const float d2 = std::fabs(encoder_q2 - drive_q2);
    channel_gap_ = d1 > d2 ? d1 : d2;

    // Once one channel has been identified and faulted there is nothing left
    // to compare, and the surviving channel must not be condemned for
    // disagreeing with a sensor already known to be broken.
    if (status_[static_cast<int>(SensorId::kEncoders)].health == Health::kFaulted ||
        status_[static_cast<int>(SensorId::kDrivePosition)].health == Health::kFaulted) {
        channels_disagree_ = false;
        disagree_run_ = 0;
        return;
    }

    if (channel_gap_ > p_.channel_disagree_rad) {
        ++disagree_run_;
        if (disagree_run_ >= p_.disagree_after) {
            channels_disagree_ = true;
            if (!arbitrating_ && !have_verdict_) {
                // Start arbitration: suspend both, record where each one
                // stands, and watch how far each claims to travel.
                arbitrating_ = true;
                arb_started_us_ = now_us;
                arb_samples_ = 0;
                arb_enc_start_[0] = encoder_q1;
                arb_enc_start_[1] = encoder_q2;
                arb_drv_start_[0] = drive_q1;
                arb_drv_start_[1] = drive_q2;
                arb_pred_delta_[0] = 0.0f;
                arb_pred_delta_[1] = 0.0f;
            }
        }
    } else {
        disagree_run_ = 0;
        channels_disagree_ = false;
    }
}

float SensorHealth::detection_ms(SensorId s) const {
    const SensorStatus& st = status_[static_cast<int>(s)];
    if (st.health != Health::kFaulted) return 0.0f;
    if (st.faulted_at_us <= st.first_fail_us) return 0.0f;
    return static_cast<float>(st.faulted_at_us - st.first_fail_us) * 1e-3f;
}

int SensorHealth::faulted_count() const {
    int n = 0;
    for (int i = 0; i < kNumSensors; ++i)
        if (status_[i].health == Health::kFaulted) ++n;
    return n;
}

int SensorHealth::excluded_count() const {
    // Faulted and probation both mean the sensor is out of the filter, and
    // the arm is equally degraded either way. The supervisor cares about
    // what is actually being used, not about which label it carries.
    int n = 0;
    for (int i = 0; i < kNumSensors; ++i)
        if (status_[i].health == Health::kFaulted ||
            status_[i].health == Health::kProbation) ++n;
    return n;
}

}  // namespace ppa
