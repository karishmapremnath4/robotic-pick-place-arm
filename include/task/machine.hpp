// The whole machine, assembled.
//
// Buses, drivers, estimator, health monitoring, safety supervisor, control
// loops and the task sequence, wired together in the order they have to run,
// with the simulated cell underneath.
//
// This exists because the running order is not arbitrary and getting it wrong
// is silent. Sensors are read, then the filter is propagated, then each
// sensor is offered to it, then health is judged, then safety decides what
// the arm is allowed to do, then the sequence decides what it wants, then the
// control loops are given the result. Shuffle any two of those and the arm
// still moves; it just acts on information that is a cycle stale, or decides
// its safety state from a filter that has not been updated yet.
//
// The six test applications each wire this up themselves, deliberately: they
// reach inside to freeze an encoder or read a covariance, and a shared
// facade would have to expose all of that anyway. This one is for the
// interactive console and for anything else that just wants a working arm.
#pragma once

#include "config.hpp"
#include "ctrl/joint_axis.hpp"
#include "diag/safety.hpp"
#include "diag/sensor_health.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "drivers/laser.hpp"
#include "est/ekf.hpp"
#include "sim/sensor_board.hpp"
#include "task/sequence.hpp"
#include "task/survey.hpp"

namespace ppa {

// Everything worth looking at from outside, gathered once per cycle so the
// console and the telemetry writer do not each go rummaging.
struct Telemetry {
    double t_s{0.0};

    // Truth, which only the simulation knows and which the firmware never
    // sees. Logged so the estimate can be graded.
    float true_q1{0.0f}, true_q2{0.0f};
    float true_x{0.0f}, true_y{0.0f};
    float true_payload_kg{0.0f};
    float true_force_n{0.0f};

    // What the firmware believes.
    float est_q1{0.0f}, est_q2{0.0f};
    float est_x{0.0f}, est_y{0.0f};
    float est_payload_kg{0.0f};
    float payload_sigma_kg{0.0f};
    float est_force_n{0.0f};
    float friction_scale{0.0f};

    float current1_a{0.0f}, current2_a{0.0f};
    float clamp_n{0.0f};
    float laser_m{0.0f};
    bool laser_valid{false};

    // Where the boxes are. Truth, and only the simulation knows it, but the
    // viewer has to draw them and a box being carried moves with the tool.
    int n_boxes{0};
    float box_x[kMaxBoxes]{};
    float box_y[kMaxBoxes]{};
    float box_half_w[kMaxBoxes]{};
    float box_half_h[kMaxBoxes]{};
    bool box_held[kMaxBoxes]{};

    TaskState task{TaskState::kIdle};
    SurveyState surveying{SurveyState::kIdle};
    SafetyState safety{SafetyState::kNormal};
    TripReason trip{TripReason::kNone};
    Health health[kNumSensors]{};
    float nis[kNumSensors]{};
};

class Machine {
public:
    explicit Machine(u64 seed = 20261007u);

    // One 1 kHz control cycle, plus the plant underneath it.
    void cycle();

    void start_job(const Job& j);

    // Sweep the table and find what is on it. While a survey runs it owns
    // the arm, because it needs the tool at a particular height tracing a
    // particular line and cannot share that with a pick.
    void start_survey();
    bool surveying() const { return !survey_.finished() &&
                                    survey_.state() != SurveyState::kIdle; }
    const Survey& survey() const { return survey_; }
    void acknowledge() { safety_.acknowledge(); }

    // Faults are reachable because breaking things on purpose is the point.
    Faults& faults() { return faults_; }

    const Telemetry& telemetry() const { return tel_; }
    Sequence& sequence() { return seq_; }
    const SensorHealth& health() const { return health_; }
    const SafetySupervisor& safety() const { return safety_; }
    const World& world() const { return world_; }
    const Ekf& ekf() const { return ekf_; }

    double time_s() const { return static_cast<double>(clk_.now_us()) * 1e-6; }

private:
    void refresh_telemetry();

    arm::PlantParams pp_{arm::default_params()};
    World world_;
    Faults faults_{};
    SimClock clk_{};
    Rng rng_;
    SensorBoard board_;

    Imu imu_;
    Encoder enc1_, enc2_;
    CanNodes can_;
    LaserRanger laser_;
    Ekf ekf_{};
    SensorHealth health_{};
    SafetySupervisor safety_{};
    Sequence seq_{};
    Survey survey_{};
    JointAxis axis_[2]{};

    bool grip_cmd_{false};
    Telemetry tel_{};
};

}  // namespace ppa
