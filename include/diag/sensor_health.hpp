// Per sensor health, and why detection and isolation are different problems.
//
// DETECTION VERSUS ISOLATION
//
// Two independent position channels can prove that something is wrong: if the
// SPI encoder says one angle and the drive says another, one of them is
// lying. What two channels cannot do is say which. That is not a gap in this
// code, it is a property of dual redundancy, and it is why safety rated
// systems that must keep running use three channels and vote two out of
// three.
//
// This machine has two position channels and a third opinion of a different
// kind: the dynamics model. The filter predicts where the arm should be from
// the torque it commanded, independently of either position sensor, and the
// normalised innovation squared of each update says how far that sensor sat
// from that prediction. So:
//
//   the dual channel detects      fast, simple, needs no model
//   the model isolates            slower, needs the filter, names the culprit
//
// Both are implemented, because the fast one buys reaction time and the slow
// one buys the ability to carry on with the healthy channel instead of
// stopping.
//
// ARBITRATION, AND WHY DETECTION ALONE IS NOT ENOUGH
//
// Detecting without isolating leaves the bad sensor still being fused, and
// that is not a small problem. A sensor that is being fused drags the
// estimate toward itself, so its own innovation stays small and it goes on
// looking healthy. Measured: a frozen encoder detected by the channel
// comparison in 23 ms, left in the filter, pulled the position estimate
// 3.77 radians off truth while its own NIS never exceeded 0.70.
//
// So a disagreement starts an arbitration. BOTH position channels are
// suspended from the filter for a short window, and over that window each
// channel is judged on HOW FAR IT MOVED rather than on where it says it is.
//
// Judging on absolute position does not work, and the first version did
// exactly that and convicted the wrong sensor. By the time a disagreement
// has persisted long enough to be believed, the liar has been fused for ten
// or more cycles and has already dragged the estimate toward itself. It
// therefore looks closer to the estimate than the honest channel does, and
// the arbitration hands down precisely the wrong verdict. Observed live: a
// frozen joint 1 encoder convicted the drive instead, the filter then ran on
// the frozen reading, and the position estimate went 0.42 m wrong.
//
// Displacement is immune to that, because it does not care where the
// estimate started. The filter's joint rates come from the drive velocities
// and the tool gyro, neither of which is a position channel, so integrating
// them over the window gives an independent answer for how far the arm
// actually moved. A frozen encoder reports no movement at all while the arm
// moves, and nothing about a contaminated starting point can disguise that.
//
// The window is the cost: for its duration the arm runs without either
// position sensor. 50 ms is long enough to separate them clearly and short
// enough that the dynamics carry the estimate alone.
//
// WHY GATING NEEDS A COUNTER
//
// A chi squared gate on a single sample is a coin toss at the margins. At a
// 99.9 percent threshold roughly one update in a thousand fails by chance,
// which at 1 kHz is one per second, and a system that faults a sensor on the
// first failure will fault every sensor within minutes of starting.
//
// So a fault is declared only after a run of consecutive failures. That
// converts a per sample false alarm rate of 1e-3 into 1e-3 raised to the
// length of the run, which is negligible, at the cost of the detection delay
// the run takes. fault_check measures that delay rather than assuming it.
#pragma once

#include "types.hpp"

namespace ppa {

enum class SensorId : u8 {
    kEncoders = 0,     // the SPI channel, both joints
    kDrivePosition,    // the CAN channel, both joints
    kDriveVelocity,
    kToolImu,          // gyro and accelerometer together
    kLaser,
    kCount,
};
constexpr int kNumSensors = static_cast<int>(SensorId::kCount);

enum class Health : u8 {
    kOk = 0,
    kSuspect,   // gate failing, not yet long enough to declare
    kFaulted,   // excluded from the filter
    kProbation, // passing again, not yet readmitted
};

// Chi squared thresholds at the 99.9 percent point, by degrees of freedom.
// A two dimensional update that exceeds 13.82 is a one in a thousand event
// if the filter's covariance is right.
constexpr float kChiSq999[4] = {0.0f, 10.83f, 13.82f, 16.27f};

struct HealthPolicy {
    // Consecutive gate failures before a sensor is declared faulted. At the
    // 1 kHz update rate this is 15 ms of sustained disagreement, and it makes
    // a chance declaration vanishingly unlikely: 1e-3 to the fifteenth.
    u32 fault_after{15};

    // Consecutive passes before a faulted sensor is readmitted. Much longer
    // than the fault threshold on purpose. Readmitting a sensor that is still
    // intermittently wrong is worse than leaving it out, because the filter
    // then alternates between two inconsistent views.
    u32 recover_after{500};

    // Direct comparison of the two position channels, independent of any
    // model. 0.02 rad is about 1.1 degrees, which is far outside their
    // combined noise of roughly 0.001 rad and far inside anything the arm
    // does in a few milliseconds.
    float channel_disagree_rad{0.02f};
    u32 disagree_after{10};

    // How long both position channels are suspended while their innovations
    // are compared against a filter neither is influencing.
    u32 arbitrate_samples{50};
};

struct SensorStatus {
    Health health{Health::kOk};
    u32 consecutive_fails{0};
    u32 consecutive_passes{0};
    float last_nis{0.0f};
    float worst_nis{0.0f};
    u32 total_rejected{0};
    u32 fault_events{0};
    TimeUs faulted_at_us{0};
    TimeUs first_fail_us{0};  // when the run of failures began
};

class SensorHealth {
public:
    explicit SensorHealth(const HealthPolicy& p = HealthPolicy{}) : p_(p) {}

    // Ask before fusing. A faulted sensor is not fused at all; a sensor on
    // probation is monitored but still not fused, because the point of
    // probation is to watch it against a filter it is not influencing.
    bool should_fuse(SensorId s) const {
        // Neither position channel is fused while they are being arbitrated.
        // Suspending both is the whole mechanism: a sensor still in the
        // filter cannot be judged by it.
        if (arbitrating_ &&
            (s == SensorId::kEncoders || s == SensorId::kDrivePosition))
            return false;
        const Health h = status_[static_cast<int>(s)].health;
        return h == Health::kOk || h == Health::kSuspect;
    }

    bool arbitrating() const { return arbitrating_; }
    SensorId arbitration_verdict() const { return verdict_; }
    bool has_verdict() const { return have_verdict_; }

    // Report the outcome of an update, whether or not it was fused. Monitoring
    // a sensor that is not being fused is the whole mechanism behind
    // recovery: its innovation is then measured against a filter it cannot
    // influence, which is a far stronger test than one it is helping to
    // shape.
    void report_nis(SensorId s, float nis, int dof, TimeUs now_us);

    // A fault the driver found on its own: a parity error, a NACK, a
    // heartbeat timeout, a serial line gone quiet. These are not statistical
    // and get no counter. The device said it was broken.
    void report_hard_fault(SensorId s, TimeUs now_us);
    void clear_hard_fault(SensorId s);

    // Direct comparison of the two position channels. Detects, and
    // deliberately does not isolate: on its own it can only say that one of
    // the two is wrong.
    // est_qd1 and est_qd2 are the filter's joint rates, which are informed
    // by the drive velocities and the tool gyro and NOT by either position
    // channel. That independence is what makes them usable as the referee.
    void report_channel_pair(float encoder_q1, float encoder_q2,
                             float drive_q1, float drive_q2,
                             float est_qd1, float est_qd2, float dt_s,
                             TimeUs now_us);

    bool channels_disagree() const { return channels_disagree_; }
    float channel_gap_rad() const { return channel_gap_; }

    Health health(SensorId s) const { return status_[static_cast<int>(s)].health; }
    const SensorStatus& status(SensorId s) const {
        return status_[static_cast<int>(s)];
    }

    // Milliseconds from the first failing sample to the fault being declared.
    // Zero if the sensor is not faulted.
    float detection_ms(SensorId s) const;

    int faulted_count() const;
    bool any_faulted() const { return faulted_count() > 0; }

    // Sensors currently out of the filter, whether faulted or on probation.
    // This is what the supervisor should look at: the arm is running on
    // reduced information in both cases.
    int excluded_count() const;
    bool any_excluded() const { return excluded_count() > 0; }

    static const char* name(SensorId s);
    static const char* health_name(Health h);

private:
    void declare_fault(SensorId s, TimeUs now_us);

    HealthPolicy p_{};
    SensorStatus status_[kNumSensors]{};
    bool channels_disagree_{false};
    float channel_gap_{0.0f};
    u32 disagree_run_{0};

    bool arbitrating_{false};
    u32 arb_samples_{0};
    TimeUs arb_started_us_{0};
    // Where each channel stood when the window opened, and how far the
    // independently measured joint rates say the arm has moved since.
    float arb_enc_start_[2]{0.0f, 0.0f};
    float arb_drv_start_[2]{0.0f, 0.0f};
    float arb_pred_delta_[2]{0.0f, 0.0f};
    SensorId verdict_{SensorId::kCount};
    bool have_verdict_{false};
};

}  // namespace ppa
