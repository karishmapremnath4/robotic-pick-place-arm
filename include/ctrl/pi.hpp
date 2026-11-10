// Proportional plus integral controller with output limits and anti windup.
//
// Every loop in this machine is a PI with a limit on its output, so this is
// written once and reused three times.
//
// Anti windup is the part that has to be right. If the output is already
// saturated and the integrator keeps accumulating, then when the error
// finally reverses the controller sits at its limit for as long as it takes
// to unwind, ignoring the setpoint entirely. On this arm that looks like the
// tool driving hard into the table for half a second after being told to
// stop, which is the kind of behaviour that breaks a gearbox.
//
// The method here is conditional integration: integrate only when doing so
// would not push an already saturated output further into saturation. It is
// cheaper than back calculation and easier to reason about, and it needs no
// extra tuning constant.
#pragma once

#include "types.hpp"

namespace ppa {

class Pi {
public:
    void configure(float kp, float ki, float out_min, float out_max) {
        kp_ = kp;
        ki_ = ki;
        min_ = out_min;
        max_ = out_max;
    }

    void reset() {
        integral_ = 0.0f;
        saturated_ = false;
    }

    // Anti windup can be switched off. It exists so the failure mode can be
    // measured rather than asserted: with it off, the integrator winds up
    // while the output is pinned and the loop then ignores a reversed
    // setpoint until it unwinds. ctrl_check times both and prints the
    // difference. Production code leaves this on.
    void set_antiwindup(bool on) { antiwindup_ = on; }

    float update(float error, float dt) {
        // Unsaturated output using the integral as it stands.
        const float trial = kp_ * error + ki_ * integral_;

        // Integrate only if it will not deepen an existing saturation.
        const bool would_deepen =
            antiwindup_ &&
            ((trial > max_ && error > 0.0f) || (trial < min_ && error < 0.0f));
        if (!would_deepen) integral_ += error * dt;

        float out = kp_ * error + ki_ * integral_;
        saturated_ = false;
        if (out > max_) { out = max_; saturated_ = true; }
        if (out < min_) { out = min_; saturated_ = true; }
        return out;
    }

    // True when the last update hit a limit. Worth surfacing rather than
    // hiding: a loop sitting on its limit is not controlling anything, and
    // the layer above needs to know that before it concludes anything from
    // the tracking error.
    bool saturated() const { return saturated_; }
    float integral() const { return integral_; }

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float min_{0.0f};
    float max_{0.0f};
    float integral_{0.0f};
    bool saturated_{false};
    bool antiwindup_{true};
};

}  // namespace ppa
