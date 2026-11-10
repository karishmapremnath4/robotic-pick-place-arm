// Finding the work, instead of being told where it is.
//
// Up to now the sequence has been handed an x coordinate and has trusted it.
// That is the same cheating step 1 was doing before the chips went in: the
// number came from the simulation, not from the machine. Real boxes are put
// on a table by a person and are never exactly where the program thinks.
//
// So the arm sweeps its laser ranger across the table and builds a height
// profile. Anything standing above the table shows up as a raised run of
// bins, and each run's centre and height is a box.
//
// This is range based object detection. It is not computer vision, there is
// no camera and no image, and calling it perception without that
// qualification would be overclaiming. What it genuinely gives the arm is
// the ability to find its own work, to notice that an expected box is
// missing, and to see something standing there that should not be.
//
// GEOMETRY THAT MATTERS
//
// The beam is mounted 30 mm along x from the tool, which is why a box the
// gripper is holding cannot blind it. That offset has to be undone when
// binning: the surface measured at a given moment is below the BEAM, not
// below the tool.
//
// The sweep height is set by the sensor's blind zone. It cannot see closer
// than 20 mm, so sweeping at 90 mm above the table leaves 50 mm of clearance
// above a 40 mm box top: comfortably inside range at both extremes.
#pragma once

#include "est/arm_model.hpp"
#include "est/ekf.hpp"
#include "types.hpp"

namespace ppa {

struct DetectedObject {
    float centre_x{0.0f};
    float top_y{0.0f};     // height of the top face, world frame
    float width_m{0.0f};
    int samples{0};        // bins it was seen in, a rough confidence
};

constexpr int kMaxObjects = 6;

// 2 mm bins. A box is 50 mm wide, so 25 bins across it, and the gaps between
// boxes are 30 mm, so 15 bins. Both are resolved with room to spare, and the
// whole grid is 160 bins of static storage.
constexpr float kBinM = 0.002f;
constexpr int kNumBins = 160;

// Height of the sweep above the table.
//
// Named rather than written twice. The first version carried the number as a
// default in this struct AND as an expression in Machine's constructor, which
// silently won: the header said 140 mm and the arm actually swept at 90 mm,
// straight through an obstacle whose top sits at 112 mm. A constant that two
// places disagree about is worse than no constant.
constexpr float kSweepAboveTableM = 0.140f;

struct SurveyConfig {
    float table_y{-0.150f};
    // Sweep height. Set by the ranger's 20 mm blind zone at one end and by
    // what the tool has to clear at the other: at 140 mm above the table the
    // beam is 140 mm from bare table, 100 mm from a box top, and 28 mm from
    // the top of the in-air obstacle, all comfortably inside range, while the
    // tool itself passes 28 mm above that obstacle instead of into it.
    float sweep_y{-0.010f};  // see kSweepAboveTableM
    float beam_from_x{0.090f};   // first beam position
    float beam_to_x{0.380f};     // last
    float step_m{0.00010f};      // 100 mm/s
    // A bin counts as part of an object once its surface stands this far
    // above the table. Larger than the ranger's noise, which the device
    // model puts at 6 mm, and far smaller than a 40 mm box.
    float object_threshold_m{0.015f};
    // Runs narrower than this are noise, not boxes.
    int min_run_bins{8};         // 16 mm

    // A step in surface height this large splits one raised run into two
    // objects.
    //
    // Standing above the table is not enough to separate things. An obstacle
    // sitting between two boxes bridges both gaps, and the profile is then
    // one continuous raised region 130 mm wide: correct segmentation of a
    // one dimensional profile, and useless to a gripper. Measured: two 50 mm
    // boxes and an obstacle reported as a single object.
    //
    // Height discontinuity separates them, because the boxes stand 40 mm
    // above the table and the obstacle 112 mm. 15 mm is far above the
    // ranger's few millimetres of noise after averaging and far below any
    // step that matters.
    float split_step_m{0.015f};
    // Approach speed for getting to the sweep start, per 1 kHz cycle.
    float approach_step_m{0.0003f};
    u32 settle_cycles{400};
    u32 timeout_cycles{12000};
};

enum class SurveyState : u8 {
    kIdle = 0,
    kMoveToStart,
    kSweeping,
    kDone,
    kFailed,
};

struct SurveyInputs {
    float q1{0.0f};
    float q2{0.0f};
    float laser_m{0.0f};
    bool laser_valid{false};
};

struct SurveyOutputs {
    float q_ref[2]{0.0f, 0.0f};
    bool pose_valid{false};
    EkfMode ekf_mode{EkfMode::kFreeMotion};
};

class Survey {
public:
    void configure(const SurveyConfig& c) { c_ = c; }

    void start();
    SurveyOutputs step(const SurveyInputs& in);

    SurveyState state() const { return state_; }
    bool finished() const {
        return state_ == SurveyState::kDone || state_ == SurveyState::kFailed;
    }

    int object_count() const { return n_objects_; }
    const DetectedObject& object(int i) const { return objects_[i]; }

    // Bins that never received a valid reading. Worth surfacing: a sweep with
    // large gaps has not actually seen the table, and concluding "no box
    // here" from no data is how an arm reaches into something.
    int bins_filled() const;

    // Fraction of the bins the sweep actually passes over that received a
    // reading. Measured against the swept range, not the whole array: the
    // array is sized generously and counting bins the beam never visits
    // reported 89 percent for a flawless sweep.
    float coverage() const;

    static const char* state_name(SurveyState s);

private:
    void analyse();

    SurveyConfig c_{};
    SurveyState state_{SurveyState::kIdle};
    u32 cycles_{0};
    float beam_x_{0.0f};
    float approach_x_{0.0f};
    bool approach_init_{false};
    u32 settled_{0};

    // Mean surface height per bin, accumulated rather than maximised.
    //
    // Keeping the highest reading biases every height upward, because the
    // maximum of several noisy samples is not an estimate of their centre.
    // Measured: box tops came out 6 to 10 mm high against a 6 mm ranger
    // noise, consistently in one direction, which is the signature of a
    // maximum rather than of noise.
    double surface_sum_[kNumBins]{};
    u32 surface_n_[kNumBins]{};

    DetectedObject objects_[kMaxObjects]{};
    int n_objects_{0};
};

}  // namespace ppa
