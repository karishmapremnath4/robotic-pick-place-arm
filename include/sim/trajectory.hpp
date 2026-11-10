// Jerk limited motion profile, and a straight line Cartesian path built on it.
//
// A trapezoidal velocity profile has a step in acceleration, which means
// infinite jerk. On a real arm that shows up as a bang at the start and end of
// every move and excites structural modes. Limiting jerk means the
// acceleration ramps instead of stepping, giving the familiar S shaped
// velocity curve.
//
// The profile is solved by bisection on the peak velocity rather than by
// enumerating the degenerate short move cases. That keeps one code path for
// every move length.
#pragma once

#include "sim/arm_dynamics.hpp"

namespace arm {

struct Limits {
    double v_max;  // units per second
    double a_max;  // units per second squared
    double j_max;  // units per second cubed
};

struct ProfilePoint {
    double s;    // position along the path, 0 to distance
    double sd;   // speed
    double sdd;  // acceleration
};

class ScurveProfile {
public:
    // Plan a rest to rest move of the given distance.
    ScurveProfile(double distance, const Limits& lim);

    double duration() const { return total_; }
    ProfilePoint at(double t) const;

private:
    double distance_{};
    double j_{};      // jerk actually used
    double a_peak_{};
    double v_peak_{};
    double tj_{};     // jerk phase duration
    double ta_{};     // constant acceleration phase duration
    double tv_{};     // constant velocity phase duration
    double total_{};

    void plan_for_peak_velocity(double v_peak);
    double distance_for_peak_velocity(double v_peak);
};

// A straight line in x and y, traversed with a jerk limited speed profile.
class CartesianLine {
public:
    CartesianLine(const Vec2& from, const Vec2& to, const Limits& lim);

    double duration() const { return profile_.duration(); }
    Vec2 point_at(double t) const;

    // Path velocity at time t. A controller that is only told WHERE to be
    // has to lag behind a moving target in order to generate any command at
    // all: a proportional position loop settles at an error of velocity
    // divided by gain. Telling it how fast to be going removes that lag
    // rather than papering over it with more gain.
    Vec2 velocity_at(double t) const;

private:
    Vec2 from_{};
    Vec2 dir_{};  // unit vector
    ScurveProfile profile_;
};

}  // namespace arm
