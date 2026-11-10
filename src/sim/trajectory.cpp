#include "sim/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace arm {

void ScurveProfile::plan_for_peak_velocity(double v_peak) {
    v_peak_ = v_peak;

    // Time to ramp acceleration from zero to a_max at the jerk limit.
    const double tj_full = a_peak_ / j_;

    if (v_peak >= a_peak_ * tj_full) {
        // Acceleration reaches a_max and holds for a while.
        tj_ = tj_full;
        ta_ = v_peak / a_peak_ - tj_full;
    } else {
        // Short move: acceleration never reaches a_max, so the profile is
        // triangular in acceleration.
        tj_ = std::sqrt(v_peak / j_);
        ta_ = 0.0;
    }
}

double ScurveProfile::distance_for_peak_velocity(double v_peak) {
    plan_for_peak_velocity(v_peak);
    // Distance covered accelerating from rest to v_peak, then the same again
    // decelerating back to rest.
    const double t_accel = 2.0 * tj_ + ta_;
    return v_peak * t_accel;
}

ScurveProfile::ScurveProfile(double distance, const Limits& lim)
    : distance_(distance), j_(lim.j_max), a_peak_(lim.a_max) {
    if (distance <= 0.0) {
        total_ = 0.0;
        return;
    }

    // Does the move have room to reach v_max?
    if (distance_for_peak_velocity(lim.v_max) <= distance) {
        plan_for_peak_velocity(lim.v_max);
        const double d_accel = distance_for_peak_velocity(lim.v_max);
        tv_ = (distance - d_accel) / lim.v_max;
    } else {
        // It does not, so find the peak velocity that exactly fits. The
        // distance is monotonic in peak velocity, so bisection is safe.
        double lo = 0.0;
        double hi = lim.v_max;
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (distance_for_peak_velocity(mid) < distance) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        plan_for_peak_velocity(0.5 * (lo + hi));
        tv_ = 0.0;
    }

    total_ = 4.0 * tj_ + 2.0 * ta_ + tv_;
}

ProfilePoint ScurveProfile::at(double t) const {
    if (total_ <= 0.0) return ProfilePoint{0.0, 0.0, 0.0};
    t = std::clamp(t, 0.0, total_);

    // Phase boundaries: jerk up, hold accel, jerk down, cruise, then mirrored.
    const double t1 = tj_;
    const double t2 = t1 + ta_;
    const double t3 = t2 + tj_;
    const double t4 = t3 + tv_;
    const double t5 = t4 + tj_;
    const double t6 = t5 + ta_;

    double s = 0.0, sd = 0.0, sdd = 0.0;

    // State at the end of each phase, accumulated so the segments join up.
    const double sd1 = 0.5 * j_ * t1 * t1;
    const double s1 = j_ * t1 * t1 * t1 / 6.0;
    const double a1 = j_ * t1;

    const double sd2 = sd1 + a1 * ta_;
    const double s2 = s1 + sd1 * ta_ + 0.5 * a1 * ta_ * ta_;

    const double s3 = s2 + sd2 * tj_ + 0.5 * a1 * tj_ * tj_
                    - j_ * tj_ * tj_ * tj_ / 6.0;
    const double s4 = s3 + v_peak_ * tv_;

    if (t < t1) {
        sdd = j_ * t;
        sd = 0.5 * j_ * t * t;
        s = j_ * t * t * t / 6.0;
    } else if (t < t2) {
        const double dt = t - t1;
        sdd = a1;
        sd = sd1 + a1 * dt;
        s = s1 + sd1 * dt + 0.5 * a1 * dt * dt;
    } else if (t < t3) {
        const double dt = t - t2;
        sdd = a1 - j_ * dt;
        sd = sd2 + a1 * dt - 0.5 * j_ * dt * dt;
        s = s2 + sd2 * dt + 0.5 * a1 * dt * dt - j_ * dt * dt * dt / 6.0;
    } else if (t < t4) {
        const double dt = t - t3;
        sdd = 0.0;
        sd = v_peak_;
        s = s3 + v_peak_ * dt;
    } else if (t < t5) {
        const double dt = t - t4;
        sdd = -j_ * dt;
        sd = v_peak_ - 0.5 * j_ * dt * dt;
        s = s4 + v_peak_ * dt - j_ * dt * dt * dt / 6.0;
    } else if (t < t6) {
        const double dt = t - t5;
        const double sd5 = v_peak_ - 0.5 * j_ * tj_ * tj_;
        const double s5 = s4 + v_peak_ * tj_ - j_ * tj_ * tj_ * tj_ / 6.0;
        sdd = -a1;
        sd = sd5 - a1 * dt;
        s = s5 + sd5 * dt - 0.5 * a1 * dt * dt;
    } else {
        // Mirror of the first jerk phase, measured back from the end.
        const double dt = total_ - t;
        sdd = -j_ * dt;
        sd = 0.5 * j_ * dt * dt;
        s = distance_ - j_ * dt * dt * dt / 6.0;
    }

    return ProfilePoint{s, sd, sdd};
}

CartesianLine::CartesianLine(const Vec2& from, const Vec2& to, const Limits& lim)
    : from_(from),
      profile_(std::hypot(to[0] - from[0], to[1] - from[1]), lim) {
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double len = std::hypot(dx, dy);
    dir_ = (len > 0.0) ? Vec2{dx / len, dy / len} : Vec2{0.0, 0.0};
}

Vec2 CartesianLine::velocity_at(double t) const {
    const ProfilePoint p = profile_.at(t);
    return Vec2{dir_[0] * p.sd, dir_[1] * p.sd};
}

Vec2 CartesianLine::point_at(double t) const {
    const ProfilePoint p = profile_.at(t);
    return Vec2{from_[0] + dir_[0] * p.s, from_[1] + dir_[1] * p.s};
}

}  // namespace arm
