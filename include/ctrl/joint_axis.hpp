// Cascaded position and velocity control for one joint.
//
// Two loops on the main controller, with the current loop underneath them
// inside the drive. Three layers, each with one job:
//
//   position  angle error      -> velocity setpoint     (P, plus feedforward)
//   velocity  velocity error   -> current setpoint      (PI)
//   current   current error    -> terminal voltage      (PI, in the drive)
//
// Why cascade rather than one big controller from angle straight to voltage.
// Each layer gets its own limit, and a limit that actually holds: the drive
// cannot be asked for more than 6 A because the velocity loop's output is
// clamped there, and the joint cannot be asked to exceed its speed limit
// because the position loop's output is clamped there. A single loop has one
// output to clamp and therefore one thing it can protect.
//
// Bandwidths are separated by roughly a factor of five per layer, so each
// inner loop looks instantaneous to the one outside it. That separation is
// what makes tuning them independently valid rather than wishful.
//
// Gravity is fed forward rather than left to integral action. Without it the
// velocity loop's integrator has to wind up to hold the arm against gravity,
// and everything that integrator does to fight gravity is time it is not
// spending on the actual tracking error.
#pragma once

#include "ctrl/pi.hpp"
#include "est/arm_model.hpp"

namespace ppa {

struct AxisGains {
    float kp_pos;   // velocity setpoint per radian of error
    float kp_vel;   // amps per rad/s of error
    float ki_vel;   // amps per rad/s per second
};

// Tuned for the fitted parts. Joint 1 carries the whole arm so it sees more
// inertia and gets less gain; joint 2 is lighter and faster.
//
// Position gain sets the closed loop bandwidth directly: 20 rad/s is about
// 3 Hz, which is a fifth of the velocity loop and a fiftieth of the current
// loop, keeping the cascade separation intact.
constexpr AxisGains kAxisGains[2] = {
    {20.0f, 3.0f, 30.0f},
    {20.0f, 2.0f, 20.0f},
};

class JointAxis {
public:
    void configure(int joint) {
        joint_ = joint;
        const DriveConstants& d = kDrive[joint];
        const AxisGains& g = kAxisGains[joint];
        vel_.configure(g.kp_vel, g.ki_vel, -d.current_limit, d.current_limit);
        kp_pos_ = g.kp_pos;
        speed_limit_ = d.speed_limit;
        current_limit_ = d.current_limit;
    }

    void reset() { vel_.reset(); }

    // Exposed so ctrl_check can measure what anti windup is worth.
    void set_antiwindup(bool on) { vel_.set_antiwindup(on); }
    float velocity_integral() const { return vel_.integral(); }

    // One update. q_ref and qd_ref come from the trajectory generator; qd_ref
    // is fed forward so the arm does not have to lag behind a moving target
    // in order to generate a velocity command.
    //
    // payload_kg is whatever the estimator currently believes is in the
    // gripper. Step 5 supplies it; until then zero is passed and the arm sags
    // slightly when loaded, which is exactly the error the estimate removes.
    float update(float q_ref, float qd_ref, float q, float qd,
                 float payload_kg, float q_other, float dt) {
        // --- position loop -------------------------------------------------
        float qd_cmd = qd_ref + kp_pos_ * (q_ref - q);
        if (qd_cmd > speed_limit_) { qd_cmd = speed_limit_; pos_sat_ = true; }
        else if (qd_cmd < -speed_limit_) { qd_cmd = -speed_limit_; pos_sat_ = true; }
        else { pos_sat_ = false; }
        qd_cmd_ = qd_cmd;

        // --- velocity loop -------------------------------------------------
        const float i_fb = vel_.update(qd_cmd - qd, dt);

        // --- gravity feedforward -------------------------------------------
        // Computed for the full pose, because joint 1's gravity load depends
        // on where joint 2 is. Feeding forward joint 1 using only its own
        // angle would be wrong by the whole contribution of link 2.
        const Vec<2> g = (joint_ == 0) ? gravity_torque(q, q_other, payload_kg)
                                       : gravity_torque(q_other, q, payload_kg);
        const float i_ff = torque_to_current(joint_, g[joint_]);
        i_ff_ = i_ff;

        float i_cmd = i_fb + i_ff;
        if (i_cmd > current_limit_) { i_cmd = current_limit_; i_sat_ = true; }
        else if (i_cmd < -current_limit_) { i_cmd = -current_limit_; i_sat_ = true; }
        else { i_sat_ = false; }
        return i_cmd;
    }

    float velocity_command() const { return qd_cmd_; }
    float gravity_current() const { return i_ff_; }
    bool position_saturated() const { return pos_sat_; }
    bool current_saturated() const { return i_sat_; }
    bool velocity_loop_saturated() const { return vel_.saturated(); }

private:
    Pi vel_{};
    int joint_{0};
    float kp_pos_{0.0f};
    float speed_limit_{0.0f};
    float current_limit_{0.0f};
    float qd_cmd_{0.0f};
    float i_ff_{0.0f};
    bool pos_sat_{false};
    bool i_sat_{false};
};

}  // namespace ppa
