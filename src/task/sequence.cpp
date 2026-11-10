#include "task/sequence.hpp"

#include <cmath>
#include <utility>

namespace ppa {

namespace {
// How close counts as arrived. 3 mrad at the joints is well under a
// millimetre at the tool, and loose enough that the position loop's own
// steady state error does not stall the sequence.
constexpr float kPoseTolRad = 0.003f;
constexpr int kMaxRetries = 2;
}  // namespace

const char* Sequence::state_name(TaskState s) {
    switch (s) {
        case TaskState::kIdle: return "idle";
        case TaskState::kMoveAbovePick: return "move above pick";
        case TaskState::kDescendToPick: return "descend to pick";
        case TaskState::kClose: return "close gripper";
        case TaskState::kVerifyGrip: return "verify grip";
        case TaskState::kLift: return "lift";
        case TaskState::kMoveAbovePlace: return "move above place";
        case TaskState::kDescendToPlace: return "descend to place";
        case TaskState::kRelease: return "release";
        case TaskState::kVerifyRelease: return "verify release";
        case TaskState::kRetract: return "retract";
        case TaskState::kRecover: return "recover";
        case TaskState::kDone: return "done";
        case TaskState::kFailed: return "FAILED";
    }
    return "?";
}

const char* Sequence::event_name(TaskEvent e) {
    switch (e) {
        case TaskEvent::kNone: return "none";
        case TaskEvent::kJobStarted: return "job started";
        case TaskEvent::kContactFound: return "contact found";
        case TaskEvent::kGripConfirmed: return "grip confirmed";
        case TaskEvent::kPickFailedEmpty: return "pick failed, gripper empty";
        case TaskEvent::kWrongMass: return "wrong item, mass mismatch";
        case TaskEvent::kBoxDroppedInTransit: return "BOX DROPPED IN TRANSIT";
        case TaskEvent::kPlaced: return "placed";
        case TaskEvent::kReleaseFailed: return "release failed, box stuck";
        case TaskEvent::kTimedOut: return "timed out";
        case TaskEvent::kAbortedBySafety: return "aborted by safety";
        case TaskEvent::kRecovered: return "recovered";
    }
    return "?";
}

void Sequence::emit(TaskEvent e) {
    if (n_pending_ < 4) pending_[n_pending_++] = e;
}

TaskEvent Sequence::take_event() {
    if (n_pending_ == 0) return TaskEvent::kNone;
    const TaskEvent e = pending_[0];
    for (int i = 1; i < n_pending_; ++i) pending_[i - 1] = pending_[i];
    --n_pending_;
    return e;
}

void Sequence::enter(TaskState s) {
    state_ = s;
    cycles_ = 0;
    traverse_init_ = false;
    lost_load_run_ = 0;
    contact_run_ = 0;
}

void Sequence::start(const Job& j) {
    job_ = j;
    retries_ = 0;
    measured_kg_ = 0.0f;
    emit(TaskEvent::kJobStarted);
    enter(TaskState::kMoveAbovePick);
}

void Sequence::abort() {
    emit(TaskEvent::kAbortedBySafety);
    enter(TaskState::kFailed);
    ++failed_;
}

bool Sequence::at_pose(float q1, float q2, float want_q1, float want_q2) const {
    return std::fabs(wrap_pi(q1 - want_q1)) < kPoseTolRad &&
           std::fabs(wrap_pi(q2 - want_q2)) < kPoseTolRad;
}

TaskOutputs Sequence::step(const TaskInputs& in) {
    TaskOutputs out{};
    out.q_ref[0] = in.q1;
    out.q_ref[1] = in.q2;
    ++cycles_;

    // Safety outranks the sequence completely. Nothing below runs while the
    // arm is stopped, and the sequence does not get to argue.
    if (in.safety == SafetyState::kSafeStop) {
        if (state_ != TaskState::kFailed && state_ != TaskState::kIdle) abort();
        out.grip_close = (state_ == TaskState::kClose ||
                          state_ == TaskState::kVerifyGrip ||
                          state_ == TaskState::kLift ||
                          state_ == TaskState::kMoveAbovePlace);
        return out;
    }

    const float travel_y = c_.table_y + c_.clearance_m;

    // Walk a Cartesian target toward x at the traverse rate, holding y.
    // Returns the pose for this cycle and whether the walk has arrived.
    auto traverse = [&](float to_x, float y) {
        const Vec<2> here = forward_kinematics(in.q1, in.q2);
        if (!traverse_init_) {
            traverse_x_ = here[0];
            traverse_init_ = true;
        }
        const float remaining = to_x - traverse_x_;
        if (std::fabs(remaining) <= c_.traverse_step_m) {
            traverse_x_ = to_x;
        } else {
            traverse_x_ += (remaining > 0.0f) ? c_.traverse_step_m
                                              : -c_.traverse_step_m;
        }
        const IkSolution s = inverse_kinematics(traverse_x_, y);
        out.pose_valid = s.reachable;
        if (s.reachable) {
            out.q_ref[0] = s.q1;
            out.q_ref[1] = s.q2;
        }
        const bool arrived = (traverse_x_ == to_x) &&
                             std::fabs(here[0] - to_x) < 0.002f &&
                             std::fabs(here[1] - y) < 0.004f;
        return std::pair<IkSolution, bool>{s, arrived};
    };

    // Has the load gone? Uses the unfiltered torque residual, because the
    // estimator's payload state is frozen in contact and too slow in free
    // motion to catch a drop before the move finishes.
    auto load_lost = [&]() {
        if (payload_at_lift_ <= c_.min_payload_kg) return false;

        // The gripper node's own report is the fastest and least ambiguous
        // signal there is. It was told to close, it confirmed it was
        // holding, and now it says it is not. Nothing was estimated and
        // nothing was filtered.
        //
        // This was not the first thing tried. The torque residual below
        // works too, and the estimator's payload state works in free motion,
        // but both are slower and the payload state is frozen outright
        // whenever the arm is in contact. Asking the gripper is better than
        // inferring from the arm.
        if (!in.gripper_holding) return true;

        // Backstop, for a grip that still reports holding while the load has
        // gone: the arm then needs roughly g times the missing mass less
        // torque than the model demands, which appears as an apparent
        // upward force.
        if (in.raw_force_y_n > c_.lost_load_force_n) {
            ++lost_load_run_;
        } else if (lost_load_run_ > 0) {
            --lost_load_run_;
        }
        return lost_load_run_ >= c_.lost_load_run;
    };

    // Contact, debounced. See contact_run in the config header.
    auto contact_felt = [&]() {
        const bool now_touching =
            in.contact_force_n > c_.contact_force_n ||
            in.raw_contact_force_n > c_.raw_trip_force_n;
        if (now_touching) ++contact_run_;
        else contact_run_ = 0;
        return contact_run_ >= c_.contact_run;
    };

    auto go = [&](float x, float y) {
        const IkSolution s = inverse_kinematics(x, y);
        out.pose_valid = s.reachable;
        if (s.reachable) {
            out.q_ref[0] = s.q1;
            out.q_ref[1] = s.q2;
        }
        return s;
    };

    switch (state_) {
        case TaskState::kIdle:
        case TaskState::kDone:
        case TaskState::kFailed:
            out.pose_valid = true;
            out.q_ref[0] = in.q1;
            out.q_ref[1] = in.q2;
            break;

        case TaskState::kMoveAbovePick: {
            const auto t = traverse(job_.pick_x, travel_y);
            if (!t.first.reachable) { emit(TaskEvent::kTimedOut); enter(TaskState::kFailed); ++failed_; break; }
            if (t.second) {
                descend_y_ = travel_y;
                enter(TaskState::kDescendToPick);
            } else if (cycles_ > c_.move_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kDescendToPick: {
            // Contact mode. The arm is about to touch something, and saying
            // so is what lets the estimator report the force instead of
            // inventing a payload. See the header.
            out.ekf_mode = EkfMode::kContact;

            // Walk the target down rather than commanding the surface
            // height, which is not known. The box may not be where the job
            // said, or may be a different height.
            //
            // Fast until nothing can be in the way, then a creep slow enough
            // that the force estimate can keep up. See the config header.
            descend_y_ -= (descend_y_ > c_.table_y + c_.creep_from_m)
                              ? c_.descend_fast_m
                              : c_.descend_creep_m;
            if (descend_y_ < c_.table_y + c_.search_floor_m) {
                // Reached the table without finding anything. There is no
                // box here.
                emit(TaskEvent::kPickFailedEmpty);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }
            go(job_.pick_x, descend_y_);

            // The filtered estimate is the better detector at creep speed;
            // the raw one is the backstop for a surface met during the fast
            // approach. See the thresholds in the config header.
            if (contact_felt()) {
                grip_y_ = descend_y_;
                emit(TaskEvent::kContactFound);
                enter(TaskState::kClose);
            } else if (cycles_ > c_.descend_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kClose: {
            out.ekf_mode = EkfMode::kContact;
            out.grip_close = true;
            go(job_.pick_x, grip_y_);
            if (in.clamp_n >= c_.clamp_target_n) {
                enter(TaskState::kVerifyGrip);
            } else if (cycles_ > c_.close_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kVerifyGrip: {
            // Back to free motion and lift slightly, because the payload is
            // only observable once the arm is carrying the box clear of the
            // surface. Verifying while still pressing on the table would be
            // asking the estimator a question it cannot answer.
            out.grip_close = true;
            go(job_.pick_x, grip_y_ + c_.place_lift_m);

            // Same as leaving a surface after placing: for the first part of
            // this lift the tool is still pressing on the box it has just
            // closed on, and declaring free motion there makes the safety
            // supervisor read a perfectly ordinary 17 N as a collision.
            const Vec<2> at = forward_kinematics(in.q1, in.q2);
            if (at[1] < grip_y_ + c_.clear_of_surface_m) {
                out.ekf_mode = EkfMode::kContact;
                break;
            }

            if (!in.payload_trustworthy) {
                if (cycles_ > c_.verify_timeout) {
                    emit(TaskEvent::kTimedOut);
                    enter(TaskState::kFailed);
                    ++failed_;
                }
                break;
            }

            measured_kg_ = in.payload_kg;

            if (measured_kg_ < c_.min_payload_kg) {
                // Closed on nothing. Worth retrying: the box may have been
                // nudged, and a retry is cheap compared to a failed job.
                if (retries_ < kMaxRetries) {
                    ++retries_;
                    out.grip_close = false;
                    enter(TaskState::kMoveAbovePick);
                } else {
                    emit(TaskEvent::kPickFailedEmpty);
                    enter(TaskState::kFailed);
                    ++failed_;
                }
                break;
            }

            if (job_.expect_kg > 0.0f &&
                std::fabs(measured_kg_ - job_.expect_kg) > job_.mass_tolerance_kg) {
                // It picked something up, just not what the job asked for.
                // Carrying on would put the wrong item in the wrong place,
                // which is worse than stopping.
                emit(TaskEvent::kWrongMass);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }

            payload_at_lift_ = measured_kg_;
            emit(TaskEvent::kGripConfirmed);
            enter(TaskState::kLift);
            break;
        }

        case TaskState::kLift: {
            out.grip_close = true;
            if (load_lost()) {
                emit(TaskEvent::kBoxDroppedInTransit);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }
            const auto t = traverse(job_.pick_x, travel_y);
            if (t.second) {
                enter(TaskState::kMoveAbovePlace);
            } else if (cycles_ > c_.move_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kMoveAbovePlace: {
            out.grip_close = true;
            const auto t = traverse(job_.place_x, travel_y);
            if (!t.first.reachable) { emit(TaskEvent::kTimedOut); enter(TaskState::kFailed); ++failed_; break; }

            // Watch the weight the whole way across. A box that was there at
            // lift and is not there now has fallen off, and that is the
            // event an operator most needs to know about, because the box is
            // on the floor and the arm is about to place nothing.
            if (load_lost() ||
                (in.payload_trustworthy &&
                 payload_at_lift_ > c_.min_payload_kg &&
                 in.payload_kg < 0.5f * payload_at_lift_)) {
                emit(TaskEvent::kBoxDroppedInTransit);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }

            if (t.second) {
                descend_y_ = travel_y;
                enter(TaskState::kDescendToPlace);
            } else if (cycles_ > c_.move_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kDescendToPlace: {
            out.ekf_mode = EkfMode::kContact;
            out.grip_close = true;
            // Keep watching for the load vanishing. The traverse is not the
            // only place a grip can fail, and in contact mode the filter's
            // payload is frozen, so only the raw residual can see it here.
            if (load_lost()) {
                emit(TaskEvent::kBoxDroppedInTransit);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }
            // Carrying something, so the creep must start higher: it is the
            // box's underside that lands, not the tool.
            descend_y_ -= (descend_y_ > c_.table_y + c_.creep_from_m +
                                            c_.carried_clearance_m)
                              ? c_.descend_fast_m
                              : c_.descend_creep_m;
            if (descend_y_ < c_.table_y + c_.search_floor_m) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }
            go(job_.place_x, descend_y_);
            if (contact_felt()) {
                grip_y_ = descend_y_;
                enter(TaskState::kRelease);
            } else if (cycles_ > c_.descend_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kRelease: {
            out.ekf_mode = EkfMode::kContact;

            // Before opening, record what was actually in the jaws.
            //
            // This is what makes a drop impossible to miss. The filter needs
            // the better part of a second to learn that a carried box has
            // gone, and a traverse can finish sooner than that: the arm then
            // reached the place position, opened an empty gripper, saw no
            // payload, and reported the box PLACED. Claiming success for a
            // job it did not do is the worst failure in this whole machine.
            //
            // Asking the question at the moment of release needs no filter
            // speed at all. Either something was there or it was not.
            if (cycles_ == 1) {
                payload_before_release_ = in.payload_trustworthy
                                              ? in.payload_kg
                                              : payload_at_lift_;
                if (in.payload_trustworthy &&
                    payload_at_lift_ > c_.min_payload_kg &&
                    in.payload_kg < c_.min_payload_kg) {
                    emit(TaskEvent::kBoxDroppedInTransit);
                    enter(TaskState::kFailed);
                    ++failed_;
                    break;
                }
            }

            out.grip_close = false;
            go(job_.place_x, grip_y_);
            if (in.clamp_n < 1.0f) {
                enter(TaskState::kVerifyRelease);
            } else if (cycles_ > c_.release_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kVerifyRelease: {
            // Lift away and check the arm got lighter. A box that did not
            // let go is still in the jaws, and the next job would try to
            // pick up a second one on top of it.
            out.grip_close = false;
            go(job_.place_x, grip_y_ + c_.place_lift_m);

            // Still in contact for the first part of the lift, and saying so
            // matters. This state was declared free motion throughout, and
            // the safety supervisor then saw 17 N of force where nothing
            // should be touching and stopped the arm, correctly, every
            // single time it put a box down. The arm had just finished
            // pressing on that surface; leaving it is not a collision.
            //
            // The payload cannot be verified in contact mode, because the
            // filter freezes it there. So the mode flips to free as soon as
            // the tool is genuinely clear, and the verification below waits
            // for the estimate that only then becomes available.
            const Vec<2> here = forward_kinematics(in.q1, in.q2);
            const bool still_near_surface =
                here[1] < grip_y_ + c_.clear_of_surface_m;
            out.ekf_mode = still_near_surface ? EkfMode::kContact
                                              : EkfMode::kFreeMotion;
            if (still_near_surface) break;

            if (!in.payload_trustworthy) {
                if (cycles_ > c_.verify_timeout) {
                    emit(TaskEvent::kTimedOut);
                    enter(TaskState::kFailed);
                    ++failed_;
                }
                break;
            }
            if (in.payload_kg > c_.min_payload_kg) {
                emit(TaskEvent::kReleaseFailed);
                enter(TaskState::kFailed);
                ++failed_;
                break;
            }
            emit(TaskEvent::kPlaced);
            enter(TaskState::kRetract);
            break;
        }

        case TaskState::kRetract: {
            const auto t = traverse(job_.place_x, travel_y);
            if (t.second) {
                ++completed_;
                enter(TaskState::kDone);
            } else if (cycles_ > c_.move_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
                ++failed_;
            }
            break;
        }

        case TaskState::kRecover: {
            // Step 6 measured why this state exists. With the drives at zero
            // during a safe stop the arm sags onto the table, so
            // acknowledging the stop and resuming immediately trips again on
            // genuine contact. Recovery declares contact, lifts straight up
            // to clearance, and only then hands back to normal operation.
            out.ekf_mode = EkfMode::kContact;
            const IkSolution s = inverse_kinematics(
                kLink1M * std::cos(in.q1) + kLink2M * std::cos(in.q1 + in.q2),
                travel_y);
            out.pose_valid = s.reachable;
            if (s.reachable) {
                out.q_ref[0] = s.q1;
                out.q_ref[1] = s.q2;
                if (at_pose(in.q1, in.q2, s.q1, s.q2)) {
                    emit(TaskEvent::kRecovered);
                    enter(TaskState::kIdle);
                }
            }
            if (cycles_ > c_.move_timeout) {
                emit(TaskEvent::kTimedOut);
                enter(TaskState::kFailed);
            }
            break;
        }
    }

    return out;
}

}  // namespace ppa
