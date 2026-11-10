#include "sim/world.hpp"

#include <cmath>

namespace ppa {

namespace {
constexpr double kG = 9.81;

}  // namespace

World::World(const arm::PlantParams& p, u64 seed)
    : params_(p), bare_(p.arm), plant_(p), rng_(seed) {
    // Three boxes on the table, deliberately different weights. The
    // estimator is supposed to be able to say which one it picked up, so the
    // masses are spaced far enough apart to be distinguishable but close
    // enough that it is not trivial.
    //
    // The heaviest sits near the continuous current rating of joint 1 at
    // full extension, which is where the drive is actually working hard.
    const double masses[kMaxBoxes] = {0.25, 0.50, 0.35};
    const double xs[kMaxBoxes] = {0.180, 0.260, 0.340};
    for (int i = 0; i < kMaxBoxes; ++i) {
        Box& b = boxes_[i];
        b.id = i + 1;
        b.mass_kg = masses[i];
        b.x = xs[i];
        // Resting on the table means the box bottom touches the table top.
        b.y = layout_.table_y + b.half_h;
        b.held = false;
        b.vy = 0.0;
    }
    box_count_ = kMaxBoxes;

    // Start the arm folded up and clear of the table rather than at the
    // singular straight out pose, which is both unrealistic and the worst
    // case for gravity.
    arm::State x0{{0.9, -1.6, 0.0, 0.0, 0.0, 0.0}};
    plant_.set_state(x0);
}

void World::command_grip(bool close) {
    grip_cmd_close_ = close;
    if (close) {
        if (grip_ == GripState::kOpen || grip_ == GripState::kOpening)
            grip_ = GripState::kClosing;
    } else {
        if (grip_ == GripState::kHolding || grip_ == GripState::kClosing)
            grip_ = GripState::kOpening;
    }
}

double World::required_grip_n(double mass_kg) const {
    // A friction grip has to carry the weight plus whatever inertial load the
    // move imposes, divided by the coefficient. Below this the box slips.
    return mass_kg * (kG + grip_p_.accel_allowance) / grip_p_.friction_coeff;
}

arm::Vec2 World::tip_velocity() const {
    const arm::Mat2 J = arm::jacobian(plant_.params().arm, plant_.q());
    const arm::Vec2 v = plant_.qd();
    return arm::Vec2{J[0][0] * v[0] + J[0][1] * v[1],
                     J[1][0] * v[0] + J[1][1] * v[1]};
}

arm::Vec2 World::tip_acceleration() const {
    const arm::ArmParams& a = plant_.params().arm;
    const arm::Vec2 q = plant_.q();
    const arm::Vec2 qd = plant_.qd();

    const double s1 = std::sin(q[0]);
    const double c1 = std::cos(q[0]);
    const double s12 = std::sin(q[0] + q[1]);
    const double c12 = std::cos(q[0] + q[1]);
    const double w12 = qd[0] + qd[1];

    // J, the same matrix arm::jacobian returns, written out here because
    // Jdot needs the same trigonometry and computing it twice would be
    // wasteful in the inner loop.
    const double J00 = -a.l1 * s1 - a.l2 * s12;
    const double J01 = -a.l2 * s12;
    const double J10 = a.l1 * c1 + a.l2 * c12;
    const double J11 = a.l2 * c12;

    // Jdot, by differentiating each entry of J with respect to time.
    const double D00 = -a.l1 * c1 * qd[0] - a.l2 * c12 * w12;
    const double D01 = -a.l2 * c12 * w12;
    const double D10 = -a.l1 * s1 * qd[0] - a.l2 * s12 * w12;
    const double D11 = -a.l2 * s12 * w12;

    return arm::Vec2{J00 * qdd_[0] + J01 * qdd_[1] + D00 * qd[0] + D01 * qd[1],
                     J10 * qdd_[0] + J11 * qdd_[1] + D10 * qd[0] + D11 * qd[1]};
}

arm::Vec2 World::tool_specific_force() const {
    const arm::Vec2 a = tip_acceleration();

    // Specific force in world axes: acceleration minus gravity. Gravity acts
    // along -y, so subtracting it adds +g to the y component.
    const double fx = a[0];
    const double fy = a[1] + kG;

    // Rotate into the tool frame, which is turned by q1 + q2.
    const double th = tool_angle();
    const double c = std::cos(th);
    const double s = std::sin(th);
    return arm::Vec2{fx * c + fy * s, -fx * s + fy * c};
}

arm::Vec2 World::resolve_contact(const Faults& f) {
    const arm::Vec2 p = plant_.tip();
    const arm::Vec2 v = tip_velocity();

    double fx = 0.0;
    double fy = 0.0;
    bool touching = false;

    // --- a carried box meeting a surface ---------------------------------
    //
    // This was missing until step 7 needed it, and its absence was not
    // harmless. With no contact between the held box and the table, the box
    // passed straight through and the tool carried on descending until the
    // TOOL hit the table, by which point it was pressing 248 N: over the
    // mechanical limit, and the safety supervisor was right to trip.
    //
    // The box is held rigidly at the tool, so a force on the box acts at the
    // tool. Its bottom face is half its height below the tool point.
    if (held_ >= 0) {
        const Box& b = boxes_[held_];
        // Top face at the tool, so the bottom is a whole height below it.
        const double bottom = p[1] - 2.0 * b.half_h;
        double surface = layout_.table_y;

        // Another box already resting underneath raises the surface.
        for (int i = 0; i < box_count_; ++i) {
            if (i == held_) continue;
            const Box& o = boxes_[i];
            if (o.held) continue;
            if (std::fabs(p[0] - o.x) > o.half_w + b.half_w) continue;
            const double top = o.y + o.half_h;
            if (top > surface) surface = top;
        }

        const double pen = surface - bottom;
        if (pen > 0.0) {
            double fn = contact_p_.k_n_m * pen - contact_p_.c_n_s_m * v[1];
            if (fn < 0.0) fn = 0.0;
            fy += fn;
            touching = true;
        }
    }

    // --- the table, against the tool itself -------------------------------
    {
        const double pen = layout_.table_y - p[1];
        if (pen > 0.0) {
            // Spring plus damper, and the normal force can only push up,
            // never pull the tool down onto the surface.
            double fn = contact_p_.k_n_m * pen - contact_p_.c_n_s_m * v[1];
            if (fn < 0.0) fn = 0.0;
            fy += fn;
            touching = true;
        }
    }

    // --- box tops, for the boxes still on the table ----------------------
    // Descending onto a box is how a pick begins: the tool feels the box
    // before the gripper closes, which is what "feel for contact" means.
    for (int i = 0; i < box_count_; ++i) {
        const Box& b = boxes_[i];
        if (b.held) continue;
        if (std::fabs(p[0] - b.x) > b.half_w) continue;
        const double top = b.y + b.half_h;
        const double pen = top - p[1];

        // Only a SHALLOW penetration counts as resting on the top face.
        //
        // The gripper holds a box with the box's centre at the tool point,
        // so while held the tool is geometrically inside the box. That is
        // fine because held boxes skip this check. But the instant the jaws
        // open, the box is no longer held and still sits exactly at the tool,
        // which made this test fire with a penetration of one half height,
        // 20 mm, and produce 3000 N from nothing. The safety supervisor
        // stopped the arm every time a box was released.
        //
        // A depth limit, because penalty contact has no notion of a solid
        // interior. Contact is detected at 4 N, which at this stiffness is
        // 0.03 mm, so a limit of one half height is enormously generous for
        // a real approach while still refusing to pretend the tool is
        // resting on a face it has passed clean through.
        if (pen > 0.0 && pen < b.half_h) {
            double fn = contact_p_.k_n_m * pen - contact_p_.c_n_s_m * v[1];
            if (fn < 0.0) fn = 0.0;
            fy += fn;
            touching = true;
        }
    }

    // --- an obstacle that should not be there -----------------------------
    if (f.obstacle_present) {
        const double dx = p[0] - layout_.obstacle_x;
        const double dy = p[1] - layout_.obstacle_y;
        const double d = std::sqrt(dx * dx + dy * dy);
        const double pen = layout_.obstacle_radius - d;
        if (pen > 0.0 && d > 1e-9) {
            const double nx = dx / d;
            const double ny = dy / d;
            const double vn = v[0] * nx + v[1] * ny;
            double fn = contact_p_.k_n_m * pen - contact_p_.c_n_s_m * vn;
            if (fn < 0.0) fn = 0.0;
            fx += fn * nx;
            fy += fn * ny;
            touching = true;
            if (!in_collision_) {
                ++collisions_;
                in_collision_ = true;
            }
        }
    }
    if (!f.obstacle_present) in_collision_ = false;

    contact_f_ = arm::Vec2{fx, fy};
    if (!touching) contact_f_ = arm::Vec2{0.0, 0.0};

    // Map the tool force to joint torques. tau = J^T * F, which is the
    // standard duality between force and torque: the same Jacobian that
    // turns joint rates into tip velocity turns tip force into joint torque.
    const arm::Mat2 J = arm::jacobian(plant_.params().arm, plant_.q());
    return arm::Vec2{J[0][0] * contact_f_[0] + J[1][0] * contact_f_[1],
                     J[0][1] * contact_f_[0] + J[1][1] * contact_f_[1]};
}

void World::update_grip(const Faults& f, double dt) {
    // Clamp force follows the command, with a leak fault that bleeds it away
    // whatever the command says.
    double target = grip_cmd_close_ ? grip_p_.force_max_n : 0.0;
    if (f.grip_leak) target = 0.0;

    const double rate = f.grip_leak ? grip_p_.leak_rate_n_s
                                    : grip_p_.close_rate_n_s;
    if (grip_force_ < target) {
        grip_force_ = std::fmin(target, grip_force_ + rate * dt);
    } else if (grip_force_ > target) {
        grip_force_ = std::fmax(target, grip_force_ - rate * dt);
    }

    const arm::Vec2 p = plant_.tip();

    // --- pick up ----------------------------------------------------------
    if (held_ < 0 && (grip_ == GripState::kClosing || grip_ == GripState::kHolding)) {
        for (int i = 0; i < box_count_; ++i) {
            Box& b = boxes_[i];
            if (b.held) continue;
            const double dx = p[0] - b.x;
            const double dy = p[1] - (b.y + b.half_h);
            if (std::sqrt(dx * dx + dy * dy) > grip_p_.grab_tolerance_m) continue;
            // Grabbed at the top face, which is where it will be carried.
            if (grip_force_ < required_grip_n(b.mass_kg)) continue;
            b.held = true;
            b.vy = 0.0;
            held_ = i;
            grip_ = GripState::kHolding;
            break;
        }
    }

    // --- lose it ----------------------------------------------------------
    if (held_ >= 0) {
        Box& b = boxes_[held_];
        const bool released = !grip_cmd_close_;
        const bool slipped = grip_force_ < required_grip_n(b.mass_kg);
        if (released || slipped) {
            b.held = false;
            b.vy = 0.0;
            if (slipped && !released) ++drops_;
            held_ = -1;
            grip_ = grip_cmd_close_ ? GripState::kClosing : GripState::kOpen;
        }
    }

    if (!grip_cmd_close_ && grip_force_ <= 0.0) grip_ = GripState::kOpen;
}

void World::update_boxes(double dt) {
    const arm::Vec2 p = plant_.tip();
    for (int i = 0; i < box_count_; ++i) {
        Box& b = boxes_[i];
        if (b.held) {
            // The box HANGS BELOW the tool, with its top face at the tool
            // point. That is where the gripper grabs it, so it is where it
            // must be carried.
            //
            // The first version carried it with its centre at the tool,
            // which put the tool geometrically inside the box. Everything
            // worked until the jaws opened: the box was then no longer
            // "held", still sat exactly at the tool, and the tool against
            // box top contact fired with a half height penetration. Placing
            // a box produced 3000 N, and lifting away afterwards dragged the
            // tool back out through the contact zone for another 1083 N.
            //
            // A vertical offset of the box's centre does not change its
            // gravity torque at all, because gravity torque depends only on
            // horizontal distance from the joint. So with_payload's
            // assumption that the payload sits at the link length still
            // holds for the quantity the estimator actually measures.
            b.x = p[0];
            b.y = p[1] - b.half_h;
            b.vy = 0.0;
            continue;
        }
        // Unsupported boxes fall until they land on the table.
        const double rest_y = layout_.table_y + b.half_h;
        if (b.y > rest_y) {
            b.vy -= kG * dt;
            b.y += b.vy * dt;
            if (b.y <= rest_y) {
                b.y = rest_y;
                b.vy = 0.0;
            }
        }
    }
}

double World::distance_below(double x, double y, const Faults& f) const {
    // Start with the table, which is always there.
    double best = y - layout_.table_y;
    if (best < 0.0) best = 0.0;

    // A box directly underneath is nearer than the table.
    for (int i = 0; i < box_count_; ++i) {
        const Box& b = boxes_[i];
        if (b.held) continue;  // carried boxes are not under the sensor
        if (std::fabs(x - b.x) > b.half_w) continue;
        const double top = b.y + b.half_h;
        if (top > y) continue;  // above the sensor, so not below it
        const double d = y - top;
        if (d < best) best = d;
    }

    // The obstacle, treated as a disc. Only the part of it directly under
    // the beam counts.
    if (f.obstacle_present) {
        const double dx = x - layout_.obstacle_x;
        if (std::fabs(dx) < layout_.obstacle_radius) {
            const double half_chord = std::sqrt(
                layout_.obstacle_radius * layout_.obstacle_radius - dx * dx);
            const double top = layout_.obstacle_y + half_chord;
            if (top <= y) {
                const double d = y - top;
                if (d < best) best = d;
            }
        }
    }

    return best;
}

double World::payload_kg() const {
    return held_ >= 0 ? boxes_[held_].mass_kg : 0.0;
}

void World::step(const Faults& f, double dt) {
    // The arm's inertial parameters follow whatever it is holding. This is
    // the effect the estimator has to detect, so it is applied before the
    // dynamics rather than after.
    plant_.set_arm(arm::with_payload(bare_, payload_kg(), bare_.l2));

    // Friction wear scales the joint friction. Applied through the motor
    // parameters so it enters the dynamics exactly where real friction does,
    // and through a setter so the integrator state survives the change.
    for (int j = 0; j < 2; ++j) {
        arm::MotorParams m = params_.motor[j];
        m.b_visc *= f.friction_mult;
        m.f_coulomb *= f.friction_mult;
        plant_.set_motor(j, m);
    }

    tau_ext_ = resolve_contact(f);
    qd_prev_ = plant_.qd();
    plant_.step(volt_, tau_ext_, dt);

    const arm::Vec2 qd_now = plant_.qd();
    qdd_ = arm::Vec2{(qd_now[0] - qd_prev_[0]) / dt,
                     (qd_now[1] - qd_prev_[1]) / dt};
    update_grip(f, dt);
    update_boxes(dt);
}

}  // namespace ppa
