// Current loop for one joint.
//
// WHERE THIS RUNS, AND WHY IT MATTERS
//
// This code is firmware, written to the same rules as the rest, but it does
// not run on the main controller. It runs inside the servo drive, on the
// drive's own microcontroller.
//
// That is not an arbitrary split. The winding time constant is L/R, which for
// 1 mH and 1 ohm is 1 ms, so the loop wants to run near 10 kHz. Now count the
// bus: a standard CAN frame carrying two data bytes is about 63 bits before
// stuffing and near 75 with it, which at 1 Mbit/s is 75 us on the wire. One
// cycle of a two axis loop needs two commands and two status replies, so
// 300 us. At 10 kHz the whole cycle budget is 100 us.
//
// A 10 kHz current loop across CAN therefore needs three times the bandwidth
// that exists. It cannot be done, and that is precisely why every real drive
// closes current internally and accepts a torque or velocity setpoint from
// the bus instead. At the 1 kHz the bus can actually sustain, the loop would
// be running at the same rate as the process it is trying to control.
//
// So: current here, velocity and position on the main controller.
//
// The loop itself is a PI with feedforward. The feedforward matters because
// the steady state voltage needed is exactly known:
//
//   V = R i + Ke N omega_joint
//
// Resistive drop plus back emf. Supplying that directly leaves the PI with
// only the modelling error to correct, which is a far smaller job than
// generating the whole voltage from scratch.
#pragma once

#include "ctrl/pi.hpp"
#include "est/arm_model.hpp"

namespace ppa {

class CurrentLoop {
public:
    void configure(int joint) {
        joint_ = joint;
        const DriveConstants& d = kDrive[joint];
        // Bandwidth target around 1 kHz for a 1 ms winding constant, so
        // kp = L * omega_c and ki = R * omega_c with omega_c = 2 pi 1000.
        // With L = 1 mH that gives kp about 6.3 V/A and ki about 6300.
        pi_.configure(6.3f, 6300.0f, -d.bus_voltage, d.bus_voltage);
    }

    void reset() { pi_.reset(); }

    // Returns the terminal voltage to apply.
    float update(float i_ref, float i_meas, float joint_speed_rad_s, float dt) {
        const DriveConstants& d = kDrive[joint_];

        // Clamp the request to the drive's own rating before anything else.
        // A drive that lets the layer above ask for more current than the
        // transistors can survive is not protecting anything.
        if (i_ref > d.current_limit) i_ref = d.current_limit;
        if (i_ref < -d.current_limit) i_ref = -d.current_limit;
        last_ref_ = i_ref;

        const float ff = d.winding_r * i_ref +
                         d.back_emf * d.gear_ratio * joint_speed_rad_s;
        const float corr = pi_.update(i_ref - i_meas, dt);

        float v = ff + corr;
        if (v > d.bus_voltage) v = d.bus_voltage;
        if (v < -d.bus_voltage) v = -d.bus_voltage;
        return v;
    }

    float last_reference() const { return last_ref_; }
    bool saturated() const { return pi_.saturated(); }

private:
    Pi pi_{};
    int joint_{0};
    float last_ref_{0.0f};
};

}  // namespace ppa
