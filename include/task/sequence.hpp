// The pick and place sequence.
//
// A state machine with a verification step after every action that could
// silently fail. The arm does not assume the gripper closed on something; it
// checks that it got heavier. It does not assume the box is still there
// halfway through a move; it checks that it has not got lighter.
//
// WHY THIS LAYER DECLARES THE ESTIMATOR'S MODE
//
// Step 5 measured what happens when the estimator is wrong about whether the
// arm is touching anything: pressing 87 N into the table while the filter
// believed it was in free space produced a confident 6.940 kg phantom
// payload. The filter cannot work this out for itself without circular
// reasoning, because the thing it would have to infer is the thing it is
// trying to measure.
//
// This layer can, and trivially: it knows whether it just commanded a
// descent onto a surface. So each state declares its own mode, and that
// declaration is as much a part of the state definition as its target pose.
//
// WHY THE TRAVERSES ARE INTERPOLATED RATHER THAN COMMANDED
//
// Handing the position loops a single target pose and letting them both run
// to it does NOT move the tool in a straight line. Each joint takes its own
// path at its own rate, and the tool sweeps an arc between the two poses.
//
// Measured: traversing at a constant 110 mm travel height from x = 0.260 to
// x = 0.120, commanded as one endpoint, swung the tool down into the table
// and hit it hard enough to generate 2628 N, almost all of it from the
// contact damping term. The safety supervisor stopped the arm, correctly.
//
// So every traverse here walks its target across in Cartesian space, one
// small step per control cycle, which keeps the tool at travel height the
// whole way.
//
// That is a straight line, not a plan. It is enough while the only thing to
// avoid is the table, and it is exactly what step 10's configuration space
// planner replaces once there are obstacles to go around.
//
// WHY EVERY STATE HAS A TIMEOUT
//
// Every transition here waits for a physical condition: contact felt, clamp
// force reached, payload confirmed. Any of those can fail to arrive, and a
// state machine that waits forever for a condition that will never come is
// indistinguishable from a hung machine. So each state carries a deadline,
// and expiry is a defined outcome rather than an oversight.
#pragma once

#include "diag/safety.hpp"
#include "est/arm_model.hpp"
#include "est/ekf.hpp"
#include "types.hpp"

namespace ppa {

enum class TaskState : u8 {
    kIdle = 0,
    kMoveAbovePick,   // traverse to clearance height above the box
    kDescendToPick,   // come down slowly until contact is felt
    kClose,           // build clamp force
    kVerifyGrip,      // did the arm get heavier?
    kLift,            // raise clear of the table
    kMoveAbovePlace,  // traverse, watching for the box to fall off
    kDescendToPlace,
    kRelease,
    kVerifyRelease,   // did the arm get lighter again?
    kRetract,
    kRecover,         // lift clear after a stop, before resuming anything
    kDone,
    kFailed,
};

// Things worth telling someone about. Step 12 forwards these to a phone.
enum class TaskEvent : u8 {
    kNone = 0,
    kJobStarted,
    kContactFound,
    kGripConfirmed,
    kPickFailedEmpty,     // closed on nothing
    kWrongMass,           // picked something, but not what was expected
    kBoxDroppedInTransit, // the worst one: it was there, now it is not
    kPlaced,
    kReleaseFailed,       // the box did not let go
    kTimedOut,
    kAbortedBySafety,
    kRecovered,
};

struct Job {
    float pick_x{0.0f};
    float place_x{0.0f};
    // Expected mass, or zero if unknown. When given, a pick that comes out
    // more than the tolerance away is reported as the wrong item rather than
    // carried on with.
    float expect_kg{0.0f};
    float mass_tolerance_kg{0.08f};
};

struct TaskConfig {
    float table_y{-0.150f};
    float clearance_m{0.110f};   // travel height, above the carried creep start
    // DESCENT IN TWO STAGES, AND WHY THE SECOND ONE IS SLOW
    //
    // The speed at which an arm may approach an unknown surface is set by
    // how fast it can notice it has arrived, not by how fast it can move.
    //
    // Measured: at 50 mm/s the arm pressed 248 N into the table before the
    // force estimate reported 4 N. That is 1.65 mm of penetration against a
    // 1.5e5 N/m contact, and 1.65 mm at 50 mm/s is 33 ms, which is how long
    // the estimator takes to build up an external torque after contact mode
    // is declared. The safety supervisor tripped on it at its 120 N limit,
    // correctly.
    //
    // So the approach is fast down to a height where nothing can be, then a
    // creep slow enough that 33 ms of overshoot is only a fraction of a
    // millimetre. At 3 mm/s that is 0.1 mm, or about 15 N.
    //
    // An earlier value of 0.4 mm per cycle, 0.4 m/s, would have been eight
    // times worse again.
    float descend_fast_m{0.00005f};   // 50 mm/s
    float descend_creep_m{0.000020f}; // 20 mm/s, with the fast detector
    // Height above the table at which the creep begins. A box top sits 40 mm
    // up, so 55 mm is clear of the tallest thing expected.
    float creep_from_m{0.055f};

    // While carrying a box, the thing that lands is the box's underside, not
    // the tool, so the creep has to begin higher by roughly the height of
    // whatever is in the jaws. Without this the arm placed a box onto
    // another box while still in the fast phase and pressed 280 N.
    float carried_clearance_m{0.045f};
    // CONTACT THRESHOLD, AND WHY IT HAS TO BE THIS LOW
    //
    // Measured at creep speed: first contact settles around 6 N, because the
    // position loop's gain is modest and it takes it a while to build torque
    // against a blocked tool. A threshold of 10 N therefore never tripped at
    // all, and the velocity loop's integrator then wound up over several
    // seconds until the force reached 359 N and the safety supervisor
    // stopped the arm.
    //
    // So the threshold must sit below the force the control loop naturally
    // produces at first touch, not merely above the sensor noise. 4 N is
    // above the estimator's roughly 1 N accuracy at these levels and well
    // below the 6 N the loop reaches on its own.
    float contact_force_n{4.0f};

    // A second, cruder trip on the unfiltered signal. The filtered estimate
    // tracks contact within about a newton at creep speed, which makes it the
    // better detector there, but it needs ~33 ms to build up and at the fast
    // approach speed that is 1.65 mm and 248 N. This catches a surface met
    // unexpectedly early, before the creep has begun.
    float raw_trip_force_n{20.0f};

    // Consecutive cycles the contact condition must hold before it counts.
    //
    // A single cycle is not evidence. Immediately after the gripper opens or
    // closes, the payload variance is reinflated on purpose, and for a few
    // milliseconds the force estimate built on it is noisy enough to cross
    // any useful threshold. Without this the arm declared contact 10 ms
    // after starting a job, 60 mm above anything, then failed the grip
    // verification and gave up on a box that was sitting right there.
    //
    // 8 ms of sustained contact is still far inside the 50 ms the creep
    // needs to build a newton, so nothing is given away.
    u32 contact_run{8};

    // A load that is lighter than believed shows up as an apparent upward
    // force of about g times the missing mass, which for a 0.5 kg box is
    // 4.9 N. 3 N catches two thirds of that box while sitting clear of the
    // raw signal's few newtons of noise, and the run length makes a chance
    // excursion irrelevant.
    float lost_load_force_n{3.0f};
    u32 lost_load_run{60};

    // How close to the table the search is allowed to get before concluding
    // there is nothing here. A box top sits 40 mm above the table, so 10 mm
    // leaves no doubt while stopping well before the tool itself touches.
    //
    // The first version allowed the search to go 5 mm BELOW the table, which
    // at the 1.5e5 N/m contact stiffness is 750 N of press. The safety
    // supervisor tripped on it, correctly, at 248 N.
    float search_floor_m{0.010f};
    float clamp_target_n{30.0f};
    float min_payload_kg{0.10f};  // below this the gripper closed on nothing
    float place_lift_m{0.060f};

    // How far above the surface counts as clear of it. Below this the arm is
    // treated as still in contact, because it is: leaving a surface takes as
    // long as the contact force needs to fall to nothing.
    float clear_of_surface_m{0.020f};

    // Traverse speed, per 1 kHz cycle. 0.3 mm per cycle is 300 mm/s, which
    // the position loops follow comfortably and which crosses the table in
    // under a second.
    float traverse_step_m{0.0003f};

    // Deadlines, in control cycles at 1 kHz.
    u32 move_timeout{4000};
    u32 descend_timeout{8000};
    u32 close_timeout{600};
    u32 verify_timeout{3000};  // the payload estimate needs the arm to move
    u32 release_timeout{800};
};

struct TaskInputs {
    float q1{0.0f};
    float q2{0.0f};
    float payload_kg{0.0f};
    bool payload_trustworthy{false};
    // The filtered estimate, used for the safety limit because it is
    // accurate, and the raw one, used for detection because it is fast.
    float contact_force_n{0.0f};
    float raw_contact_force_n{0.0f};
    float raw_force_y_n{0.0f};  // signed, positive up
    bool gripper_holding{false};
    float clamp_n{0.0f};
    SafetyState safety{SafetyState::kNormal};
};

struct TaskOutputs {
    float q_ref[2]{0.0f, 0.0f};
    bool grip_close{false};
    EkfMode ekf_mode{EkfMode::kFreeMotion};
    bool pose_valid{false};
};

class Sequence {
public:
    void configure(const TaskConfig& c) { c_ = c; }

    void start(const Job& j);
    void abort();

    // One control cycle. Returns what the arm should do next.
    TaskOutputs step(const TaskInputs& in);

    TaskState state() const { return state_; }
    TaskEvent take_event();  // one shot, returns kNone when drained
    u32 jobs_completed() const { return completed_; }
    u32 jobs_failed() const { return failed_; }
    float last_measured_kg() const { return measured_kg_; }
    u32 cycles_in_state() const { return cycles_; }

    static const char* state_name(TaskState s);
    static const char* event_name(TaskEvent e);

private:
    void enter(TaskState s);
    void emit(TaskEvent e);
    bool at_pose(float q1, float q2, float want_q1, float want_q2) const;

    TaskConfig c_{};
    Job job_{};
    TaskState state_{TaskState::kIdle};
    u32 cycles_{0};

    // The descent target, walked down a fraction of a millimetre per cycle
    // rather than commanded straight to an unknown surface height.
    float descend_y_{0.0f};
    // Cartesian target walked across during a traverse, so the tool holds
    // its height instead of swinging.
    float traverse_x_{0.0f};
    bool traverse_init_{false};
    u32 lost_load_run_{0};
    u32 contact_run_{0};
    float grip_y_{0.0f};
    float payload_at_lift_{0.0f};
    float payload_before_release_{0.0f};
    float measured_kg_{0.0f};

    TaskEvent pending_[4]{};
    int n_pending_{0};

    u32 completed_{0};
    u32 failed_{0};
    u32 retries_{0};
};

}  // namespace ppa
