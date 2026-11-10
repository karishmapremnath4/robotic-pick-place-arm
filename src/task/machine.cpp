#include "task/machine.hpp"

#include <cmath>

namespace ppa {

namespace {
constexpr double kDt = 1.0e-5;         // plant step
constexpr int kStepsPerControl = 100;  // so the control loop runs at 1 kHz
constexpr float kControlDt = 1.0e-3f;
}  // namespace

Machine::Machine(u64 seed)
    : world_(pp_, seed),
      rng_(seed ^ 0x5DEECE66Dull),
      board_(world_, faults_, clk_, rng_),
      imu_(board_.imu_bus),
      enc1_(board_.enc1_bus, clk_),
      enc2_(board_.enc2_bus, clk_),
      can_(board_.can_bus, clk_),
      laser_(board_.laser_bus, clk_) {
    imu_.init();
    enc1_.init();
    enc2_.init();
    axis_[0].configure(0);
    axis_[1].configure(1);

    TaskConfig tc{};
    tc.table_y = static_cast<float>(world_.layout().table_y);
    seq_.configure(tc);

    SurveyConfig sc{};
    sc.table_y = tc.table_y;
    sc.sweep_y = tc.table_y + kSweepAboveTableM;
    survey_.configure(sc);

    // Seed the filter from the encoders, which is all a real machine has at
    // power on.
    const EncoderSample s1 = enc1_.read();
    const EncoderSample s2 = enc2_.read();
    ekf_.init(wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]),
              wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]));
}

void Machine::start_job(const Job& j) { seq_.start(j); }

void Machine::start_survey() { survey_.start(); }

void Machine::cycle() {
    const TimeUs now = clk_.now_us();

    // ---- read everything -------------------------------------------------
    can_.poll();
    laser_.poll();
    const EncoderSample s1 = enc1_.read();
    const EncoderSample s2 = enc2_.read();
    const ImuSample im = imu_.read();

    // ---- propagate -------------------------------------------------------
    Vec<2> tau_motor{};
    tau_motor[0] = current_to_torque(0, can_.drive(0).current_a);
    tau_motor[1] = current_to_torque(1, can_.drive(1).current_a);
    ekf_.predict(tau_motor, kControlDt);

    // ---- faults the devices admit to -------------------------------------
    if (!s1.valid || !s2.valid) health_.report_hard_fault(SensorId::kEncoders, now);
    else health_.clear_hard_fault(SensorId::kEncoders);
    if (laser_.timed_out()) health_.report_hard_fault(SensorId::kLaser, now);
    else health_.clear_hard_fault(SensorId::kLaser);

    // ---- offer each sensor to the filter ---------------------------------
    float eq1 = 0.0f, eq2 = 0.0f;
    if (s1.valid && s2.valid) {
        eq1 = wrap_pi(s1.single_turn_rad - kEncoderZeroOffsetRad[0]);
        eq2 = wrap_pi(s2.single_turn_rad - kEncoderZeroOffsetRad[1]);
        tel_.nis[0] = ekf_.update_encoders(eq1, eq2,
                                           health_.should_fuse(SensorId::kEncoders));
        health_.report_nis(SensorId::kEncoders, tel_.nis[0], 2, now);
    }
    if (can_.drive(0).valid && can_.drive(1).valid) {
        tel_.nis[1] = ekf_.update_drive_position(
            can_.drive(0).position_rad, can_.drive(1).position_rad,
            health_.should_fuse(SensorId::kDrivePosition));
        health_.report_nis(SensorId::kDrivePosition, tel_.nis[1], 2, now);

        tel_.nis[2] = ekf_.update_drive_velocity(
            can_.drive(0).velocity_rad_s, can_.drive(1).velocity_rad_s,
            health_.should_fuse(SensorId::kDriveVelocity));
        health_.report_nis(SensorId::kDriveVelocity, tel_.nis[2], 2, now);

        if (s1.valid && s2.valid)
            health_.report_channel_pair(eq1, eq2, can_.drive(0).position_rad,
                                        can_.drive(1).position_rad, ekf_.qd1(),
                                        ekf_.qd2(), kControlDt, now);
    }
    if (im.valid) {
        const bool f = health_.should_fuse(SensorId::kToolImu);
        ekf_.update_gyro(im.gyro_z, f);
        tel_.nis[3] = ekf_.update_tool_accel(im.accel_x, im.accel_y, f);
        health_.report_nis(SensorId::kToolImu, tel_.nis[3], 2, now);
    }

    // ---- what the firmware now believes about contact --------------------
    Vec<2> force{};
    const bool have_force = ekf_.contact_force(&force);
    const float force_mag =
        have_force ? std::sqrt(force[0] * force[0] + force[1] * force[1]) : 0.0f;
    const RawContact rc = raw_contact(
        ekf_.q1(), ekf_.q2(), ekf_.qd1(), ekf_.qd2(), tau_motor[0], tau_motor[1],
        ekf_.payload_trustworthy() ? ekf_.payload_kg() : 0.0f,
        ekf_.friction_scale());

    // ---- safety, before anything is allowed to act -----------------------
    SafetyInputs si{};
    si.drive_node_lost[0] = can_.node_lost(kNodeDrive1);
    si.drive_node_lost[1] = can_.node_lost(kNodeDrive2);
    si.gripper_node_lost = can_.node_lost(kNodeGripper);
    si.channels_disagree = health_.channels_disagree();
    si.contact_force_n = force_mag;
    si.in_contact_mode = (ekf_.mode() == EkfMode::kContact);
    si.payload_kg = ekf_.payload_kg();
    si.gripper_holding = (can_.gripper().state == GripReport::kHolding);
    si.grip_commanded_closed = grip_cmd_;
    si.payload_sigma_kg = ekf_.payload_sigma();
    si.tau_sigma_nm = ekf_.tau_ext_sigma(0);
    si.payload_trustworthy = ekf_.payload_trustworthy();
    safety_.update(si, health_, now);

    // ---- the survey, when one is running ---------------------------------
    // A survey owns the arm while it runs. Letting the sequence also issue
    // setpoints would have two layers fighting over the same joints.
    if (surveying()) {
        SurveyInputs sin{};
        sin.q1 = ekf_.q1();
        sin.q2 = ekf_.q2();
        sin.laser_m = laser_.sample().distance_m;
        sin.laser_valid = laser_.sample().valid;
        const SurveyOutputs sout = survey_.step(sin);
        ekf_.set_mode(sout.ekf_mode);

        const float sc = safety_.speed_scale();
        float j0 = 0.0f, j1 = 0.0f;
        if (safety_.motion_allowed() && sout.pose_valid) {
            j0 = axis_[0].update(sout.q_ref[0], 0.0f, ekf_.q1(), ekf_.qd1(),
                                 0.0f, ekf_.q2(), kControlDt) * sc;
            j1 = axis_[1].update(sout.q_ref[1], 0.0f, ekf_.q2(), ekf_.qd2(),
                                 0.0f, ekf_.q1(), kControlDt) * sc;
        }
        can_.send_drive_current(j0, j1);
        refresh_telemetry();
        for (int i = 0; i < kStepsPerControl; ++i) {
            world_.step(faults_, kDt);
            clk_.advance_us(10);
            board_.update();
        }
        return;
    }

    // ---- the sequence ----------------------------------------------------
    TaskInputs ti{};
    ti.q1 = ekf_.q1();
    ti.q2 = ekf_.q2();
    ti.payload_kg = ekf_.payload_kg();
    ti.payload_trustworthy = ekf_.payload_trustworthy();
    ti.contact_force_n = force_mag;
    ti.raw_contact_force_n = rc.valid ? rc.force_n : 0.0f;
    ti.raw_force_y_n = rc.valid ? rc.force_y_n : 0.0f;
    ti.gripper_holding = si.gripper_holding;
    ti.clamp_n = can_.gripper().clamp_n;
    ti.safety = safety_.state();
    const TaskOutputs to = seq_.step(ti);

    // The sequence declares the estimator's regime. It is the only layer that
    // can: the filter would be inferring the thing it is trying to measure.
    ekf_.set_mode(to.ekf_mode);

    if (to.grip_close != grip_cmd_) {
        grip_cmd_ = to.grip_close;
        can_.send_grip(grip_cmd_);
        ekf_.announce_grip_change();
    }

    // ---- control ---------------------------------------------------------
    const float scale = safety_.speed_scale();
    const float p = ekf_.payload_trustworthy() ? ekf_.payload_kg() : 0.0f;
    float i0 = 0.0f, i1 = 0.0f;
    if (safety_.motion_allowed() && to.pose_valid) {
        i0 = axis_[0].update(to.q_ref[0], 0.0f, ekf_.q1(), ekf_.qd1(), p,
                             ekf_.q2(), kControlDt) * scale;
        i1 = axis_[1].update(to.q_ref[1], 0.0f, ekf_.q2(), ekf_.qd2(), p,
                             ekf_.q1(), kControlDt) * scale;
    }
    can_.send_drive_current(i0, i1);

    refresh_telemetry();

    // ---- the plant -------------------------------------------------------
    for (int i = 0; i < kStepsPerControl; ++i) {
        world_.step(faults_, kDt);
        clk_.advance_us(10);
        board_.update();
    }
}

void Machine::refresh_telemetry() {
    tel_.t_s = time_s();

    const arm::Vec2 q = world_.q();
    const arm::Vec2 tip = world_.tip();
    tel_.true_q1 = static_cast<float>(q[0]);
    tel_.true_q2 = static_cast<float>(q[1]);
    tel_.true_x = static_cast<float>(tip[0]);
    tel_.true_y = static_cast<float>(tip[1]);
    tel_.true_payload_kg = static_cast<float>(world_.payload_kg());
    tel_.true_force_n = static_cast<float>(
        std::hypot(world_.contact_force()[0], world_.contact_force()[1]));

    tel_.est_q1 = ekf_.q1();
    tel_.est_q2 = ekf_.q2();
    const Vec<2> p = forward_kinematics(ekf_.q1(), ekf_.q2());
    tel_.est_x = p[0];
    tel_.est_y = p[1];
    tel_.est_payload_kg = ekf_.payload_kg();
    tel_.payload_sigma_kg = ekf_.payload_sigma();
    tel_.friction_scale = ekf_.friction_scale();

    Vec<2> f{};
    tel_.est_force_n = ekf_.contact_force(&f)
                           ? std::sqrt(f[0] * f[0] + f[1] * f[1])
                           : 0.0f;

    tel_.current1_a = static_cast<float>(world_.current()[0]);
    tel_.current2_a = static_cast<float>(world_.current()[1]);
    tel_.clamp_n = can_.gripper().clamp_n;
    tel_.laser_m = laser_.sample().valid ? laser_.sample().distance_m : 0.0f;
    tel_.laser_valid = laser_.sample().valid;
    tel_.surveying = survey_.state();

    tel_.n_boxes = world_.box_count();
    for (int i = 0; i < tel_.n_boxes && i < kMaxBoxes; ++i) {
        const Box& b = world_.box(i);
        tel_.box_x[i] = static_cast<float>(b.x);
        tel_.box_y[i] = static_cast<float>(b.y);
        tel_.box_half_w[i] = static_cast<float>(b.half_w);
        tel_.box_half_h[i] = static_cast<float>(b.half_h);
        tel_.box_held[i] = b.held;
    }

    tel_.task = seq_.state();
    tel_.safety = safety_.state();
    tel_.trip = safety_.reason();
    for (int i = 0; i < kNumSensors; ++i)
        tel_.health[i] = health_.health(static_cast<SensorId>(i));
}

}  // namespace ppa
