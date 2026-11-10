// The cell: the arm, a table, boxes, a gripper, and contact between them.
//
// This is ground truth. It is the thing the estimator is graded against and
// it is never cross compiled to the target; on a real machine its place is
// taken by the workcell.
//
// Coordinates: the arm base is the origin, x runs out along the table, y is
// up, gravity acts along -y. The table top sits below the base so the arm
// reaches down and out to it, which is the usual bench layout.
//
// Contact is resolved as a penalty: penetration produces a stiff spring and
// damper force, which is then mapped to joint torques through the transpose
// of the Jacobian. That is the standard way to get contact into a manipulator
// equation without a constraint solver, and at a 10 us step it is stable.
#pragma once

#include <array>

#include "sim/kinematics.hpp"
#include "sim/payload.hpp"
#include "sim/plant.hpp"
#include "sim/rng.hpp"
#include "sim/faults.hpp"

namespace ppa {

constexpr int kMaxBoxes = 3;

struct Box {
    int id{0};
    double x{0.0};        // centre of mass, m
    double y{0.0};
    double mass_kg{0.0};
    double half_w{0.025};
    double half_h{0.020};
    bool held{false};
    double vy{0.0};       // fall speed while unsupported
};

enum class GripState { kOpen, kClosing, kHolding, kOpening };

struct CellLayout {
    double table_y{-0.150};        // table top height, m
    double obstacle_x{0.300};      // unexpected object, when injected
    double obstacle_y{-0.060};
    double obstacle_radius{0.022};
};

// What the gripper can do, and what it takes to hold a box.
struct GripParams {
    double force_max_n{40.0};       // clamp force with the jaws fully closed
    double close_rate_n_s{300.0};   // how fast the clamp builds
    double leak_rate_n_s{60.0};     // bleed rate under the leak fault
    double friction_coeff{0.40};    // jaw to box, for a friction grip
    double accel_allowance{3.0};    // m/s^2 the grip must survive
    double grab_tolerance_m{0.012}; // how close the tool must be to grab
};

// Contact stiffness.
//
// Penalty contact always penetrates a little: the surface only pushes back in
// proportion to how far it has been intruded on. So the stiffness has to be
// chosen against the tolerance that matters, which here is the 12 mm the
// gripper needs to find a box. At 2e4 N/m a hard 90 N push sank the tool
// 4.4 mm into the table, more than a third of that budget, which is too much
// to call a numerical artifact.
//
// At 1.5e5 N/m the same push penetrates 0.6 mm and a normal 15 N working
// contact penetrates 0.1 mm. The contact resonance moves to about 62 Hz
// against roughly a kilogram of effective tip mass, still three orders of
// magnitude below the 10 us step, so nothing about stability changes. The
// step is set by the 1 ms winding time constant, not by contact.
struct ContactParams {
    double k_n_m{1.5e5};
    double c_n_s_m{300.0};
};

class World {
public:
    World(const arm::PlantParams& p, u64 seed);

    // --- commands the firmware issues ------------------------------------
    void set_voltage(double v1, double v2) { volt_ = {v1, v2}; }
    void command_grip(bool close);

    // Advance the whole cell by dt.
    void step(const Faults& f, double dt);

    // --- ground truth -----------------------------------------------------
    arm::Vec2 q() const { return plant_.q(); }
    arm::Vec2 qd() const { return plant_.qd(); }
    arm::Vec2 current() const { return plant_.current(); }
    arm::Vec2 tip() const { return plant_.tip(); }
    arm::Vec2 tip_velocity() const;

    // Tool orientation and rotation rate. For a two link planar arm the tool
    // frame is simply carried round by both joints, so its angle is q1 + q2
    // and its rate is qd1 + qd2. This is what the gyro on the end effector
    // measures.
    double tool_angle() const { return plant_.q()[0] + plant_.q()[1]; }
    double tool_rate() const { return plant_.qd()[0] + plant_.qd()[1]; }

    // Tool acceleration in world coordinates.
    //
    //   a = d/dt (J qd) = J qdd + Jdot qd
    //
    // The second term is the one people forget. It is the centripetal and
    // Coriolis part of the tool's motion, and dropping it would make the
    // simulated accelerometer disagree with the arm during any fast move,
    // which is exactly when the firmware most wants to trust it.
    arm::Vec2 tip_acceleration() const;

    // What an accelerometer bolted to the tool actually reads: specific
    // force, in the tool's own rotating frame.
    //
    // Two things trip people up here. An accelerometer measures a - g, not a,
    // so a stationary tool reads +9.81 upward rather than zero. And the
    // reading is in the tool frame, so it rotates with the wrist.
    arm::Vec2 tool_specific_force() const;

    // Torque the outside world is applying at each joint. This is what the
    // estimator has to recover without a force sensor.
    const arm::Vec2& tau_ext() const { return tau_ext_; }

    // Contact force on the tool in world coordinates, the physical quantity
    // behind tau_ext.
    const arm::Vec2& contact_force() const { return contact_f_; }

    // Distance straight down from a point to the first surface below it.
    // Boxes on the table, the table itself, and the obstacle when injected.
    // Returns a large number when nothing is below within sensor range.
    double distance_below(double x, double y, const Faults& f) const;

    double payload_kg() const;
    GripState grip_state() const { return grip_; }
    double grip_force_n() const { return grip_force_; }
    int held_box() const { return held_; }

    const Box& box(int i) const { return boxes_[i]; }
    int box_count() const { return box_count_; }
    const CellLayout& layout() const { return layout_; }
    const arm::ArmParams& bare_arm() const { return bare_; }

    // Diagnostics the simulation knows and the firmware must not see.
    u32 drop_events() const { return drops_; }
    u32 collision_events() const { return collisions_; }

private:
    arm::Vec2 resolve_contact(const Faults& f);
    void update_grip(const Faults& f, double dt);
    void update_boxes(double dt);
    double required_grip_n(double mass_kg) const;

    arm::PlantParams params_;
    arm::ArmParams bare_;
    arm::Plant plant_;
    CellLayout layout_{};
    GripParams grip_p_{};
    ContactParams contact_p_{};
    Rng rng_;

    std::array<double, 2> volt_{{0.0, 0.0}};
    arm::Vec2 tau_ext_{{0.0, 0.0}};
    arm::Vec2 contact_f_{{0.0, 0.0}};

    // Joint acceleration, needed for the tool accelerometer. Taken as the
    // change in joint velocity across the last step rather than re evaluated
    // from the dynamics: at a 10 us step the joint velocities are smooth over
    // many steps, so the difference is accurate, and it avoids duplicating
    // the manipulator equation outside the plant where the two copies could
    // drift apart.
    arm::Vec2 qdd_{{0.0, 0.0}};
    arm::Vec2 qd_prev_{{0.0, 0.0}};

    Box boxes_[kMaxBoxes]{};
    int box_count_{0};
    int held_{-1};

    GripState grip_{GripState::kOpen};
    bool grip_cmd_close_{false};
    double grip_force_{0.0};

    u32 drops_{0};
    u32 collisions_{0};
    bool in_collision_{false};
};

}  // namespace ppa
