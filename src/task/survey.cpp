#include "task/survey.hpp"

#include <cmath>

namespace ppa {

const char* Survey::state_name(SurveyState s) {
    switch (s) {
        case SurveyState::kIdle: return "idle";
        case SurveyState::kMoveToStart: return "move to sweep start";
        case SurveyState::kSweeping: return "sweeping";
        case SurveyState::kDone: return "survey done";
        case SurveyState::kFailed: return "survey FAILED";
    }
    return "?";
}

void Survey::start() {
    for (int i = 0; i < kNumBins; ++i) {
        surface_sum_[i] = 0.0;
        surface_n_[i] = 0;
    }
    n_objects_ = 0;
    beam_x_ = c_.beam_from_x;
    approach_init_ = false;
    cycles_ = 0;
    state_ = SurveyState::kMoveToStart;
}

int Survey::bins_filled() const {
    int n = 0;
    for (int i = 0; i < kNumBins; ++i)
        if (surface_n_[i] > 0) ++n;
    return n;
}

float Survey::coverage() const {
    const int span = static_cast<int>((c_.beam_to_x - c_.beam_from_x) / kBinM);
    const int n = (span < kNumBins) ? span : kNumBins;
    if (n <= 0) return 0.0f;
    int filled = 0;
    for (int i = 0; i < n; ++i)
        if (surface_n_[i] > 0) ++filled;
    return static_cast<float>(filled) / static_cast<float>(n);
}

SurveyOutputs Survey::step(const SurveyInputs& in) {
    SurveyOutputs out{};
    out.q_ref[0] = in.q1;
    out.q_ref[1] = in.q2;
    out.pose_valid = true;
    ++cycles_;

    // The tool has to sit 30 mm short of where the beam should land.
    auto aim_beam_at = [&](float beam_x) {
        const IkSolution s =
            inverse_kinematics(beam_x - kLaserOffsetXM, c_.sweep_y);
        out.pose_valid = s.reachable;
        if (s.reachable) {
            out.q_ref[0] = s.q1;
            out.q_ref[1] = s.q2;
        }
        return s;
    };

    switch (state_) {
        case SurveyState::kIdle:
        case SurveyState::kDone:
        case SurveyState::kFailed:
            break;

        case SurveyState::kMoveToStart: {
            // Walked across in Cartesian space, not commanded as an
            // endpoint. Handing the position loops a single far away pose
            // makes each joint take its own path and the tool sweep an arc
            // between them, and that arc goes wherever it likes. With an
            // obstacle on the table it went straight through it: the
            // collision detector stopped the arm before the survey had
            // sampled a single bin.
            //
            // This is the same mistake the sequence's traverses made in
            // step 7 and the same fix, which is a fair sign that moving in a
            // straight line deserves to be one piece of code rather than
            // three. Step 10's planner is where that belongs.
            const Vec<2> here = forward_kinematics(in.q1, in.q2);
            if (!approach_init_) {
                approach_x_ = here[0] + kLaserOffsetXM;
                approach_init_ = true;
            }
            const float remaining = c_.beam_from_x - approach_x_;
            if (std::fabs(remaining) <= c_.approach_step_m) {
                approach_x_ = c_.beam_from_x;
            } else {
                approach_x_ += (remaining > 0.0f) ? c_.approach_step_m
                                                  : -c_.approach_step_m;
            }

            const IkSolution s = aim_beam_at(approach_x_);
            if (!s.reachable) { state_ = SurveyState::kFailed; break; }

            // Settle before sampling. A sweep that starts while the arm is
            // still swinging bins readings against the wrong x.
            const bool arrived = (approach_x_ == c_.beam_from_x) &&
                                 std::fabs(here[0] + kLaserOffsetXM -
                                           c_.beam_from_x) < 0.003f;
            if (arrived) ++settled_;
            else settled_ = 0;
            if (settled_ > c_.settle_cycles) {
                cycles_ = 0;
                settled_ = 0;
                state_ = SurveyState::kSweeping;
            }
            if (cycles_ > c_.timeout_cycles) state_ = SurveyState::kFailed;
            break;
        }

        case SurveyState::kSweeping: {
            beam_x_ += c_.step_m;
            if (beam_x_ >= c_.beam_to_x) {
                analyse();
                    // Coverage is the gate, not the object count. A sweep that
                // saw nothing must FAIL, because "no data" is not the same
                // as "nothing there", and a sequence handed an empty list
                // would treat it as fact and reach into whatever is actually
                // sitting on the table.
                state_ = (coverage() > 0.8f) ? SurveyState::kDone
                                             : SurveyState::kFailed;
                break;
            }
            const IkSolution s = aim_beam_at(beam_x_);
            if (!s.reachable) { state_ = SurveyState::kFailed; break; }

            // Bin against where the beam ACTUALLY is, from the measured
            // pose, not where it was commanded to be. The arm lags its
            // setpoint, and binning against the command smears every edge by
            // the following error.
            if (in.laser_valid) {
                const Vec<2> tool = forward_kinematics(in.q1, in.q2);
                const float beam = tool[0] + kLaserOffsetXM;
                const int bin = static_cast<int>(
                    (beam - c_.beam_from_x) / kBinM);
                if (bin >= 0 && bin < kNumBins) {
                    surface_sum_[bin] += tool[1] - in.laser_m;
                    ++surface_n_[bin];
                }
            }

            if (cycles_ > c_.timeout_cycles) state_ = SurveyState::kFailed;
            break;
        }
    }

    return out;
}

void Survey::analyse() {
    n_objects_ = 0;

    auto height = [&](int i) {
        return static_cast<float>(surface_sum_[i] / surface_n_[i]);
    };
    auto raised_at = [&](int i) {
        return i >= 0 && i < kNumBins && surface_n_[i] > 0 &&
               height(i) > c_.table_y + c_.object_threshold_m;
    };

    // Emit one object from a run of bins, if it is wide enough to be real.
    auto emit_run = [&](int from, int to) {
        const int len = to - from;
        if (len < c_.min_run_bins || n_objects_ >= kMaxObjects) return;
        double sum = 0.0;
        int n = 0;
        for (int k = from; k < to; ++k) {
            if (surface_n_[k] == 0) continue;
            sum += height(k);
            ++n;
        }
        if (n == 0) return;
        DetectedObject& o = objects_[n_objects_++];
        o.centre_x = c_.beam_from_x +
                     kBinM * (static_cast<float>(from) +
                              0.5f * static_cast<float>(len));
        // The mean, not the maximum. A single noisy bin must not set the
        // height the arm then descends to.
        o.top_y = static_cast<float>(sum / n);
        o.width_m = kBinM * static_cast<float>(len);
        o.samples = len;
    };

    int run_start = -1;
    int seg_start = -1;
    for (int i = 0; i <= kNumBins; ++i) {
        const bool raised = raised_at(i);

        if (raised && run_start < 0) {
            run_start = i;
            seg_start = i;
        } else if (raised && run_start >= 0) {
            // Within a run, a cliff in surface height means two different
            // things are standing next to each other.
            if (raised_at(i - 1) &&
                std::fabs(height(i) - height(i - 1)) > c_.split_step_m) {
                emit_run(seg_start, i);
                seg_start = i;
            }
        } else if (!raised && run_start >= 0) {
            emit_run(seg_start, i);
            run_start = -1;
            seg_start = -1;
        }
    }
}

}  // namespace ppa
