// Planning in configuration space.
//
// WHY THIS IS CHEAP FOR THIS ARM
//
// The arm has two joints, so the set of all poses it can hold is a two
// dimensional square: one axis per joint. Every pose the arm can be in is a
// single point on that square, and every obstacle in the room maps to some
// region of it that the arm must not enter.
//
// That makes the hard part of motion planning disappear. There is no need for
// sampling, trees or randomness. Grid the square, mark which cells would put
// some part of the arm through something solid, and search the rest.
//
// The map is NOT the room. A box 50 mm wide on the table becomes a curved
// band across this square, because many different pairs of joint angles put
// the arm through that box. That is the thing worth understanding about
// configuration space, and drawing it is the clearest way to see it.
//
// WHY IT IS NEEDED, AND A CORRECTION
//
// A straight line in joint space between two poses does not move the tool in
// a straight line: each joint takes its own path and the tool sweeps an arc
// between them. How far that arc strays is the question, and this project
// answered it twice, wrongly the first time.
//
// The claim made in step 7 was that traversing at constant height swung the
// tool down into the table and generated 2628 N. plan_check measures it: the
// straight joint space line from x = 0.260 to x = 0.120, both at 110 mm above
// the table, dips the tool to 96 mm above the table. It sags 14 mm. It does
// not come close to the table, and it never could have: the 2628 N was
// recorded with the tool at 110 mm of clearance.
//
// That force came from the held box geometry bug found later in step 7, where
// a box carried with its centre at the tool put the tool inside the box and
// the contact test fired with a half height penetration. The traverse was
// innocent, and the Cartesian interpolation added to "fix" it fixed nothing.
// It is harmless and still in place, but it was not the cure.
//
// Where the arc DOES matter is an obstacle standing up into the travel
// height, which step 9 hit for real: moving to the sweep start swept through
// an obstacle whose top sits 2 mm above the travel plane. plan_check
// demonstrates that case, and it is the honest justification for planning.
//
// The broader reason stands on its own. Interpolating each move separately,
// in three different places, encodes "avoid the table" three times and
// "avoid anything else" nowhere. One map of what the arm may not do, built
// from what the arm actually saw, replaces all of it.
//
// WHAT IT DOES NOT DO
//
// The grid is 3.75 degrees per cell, which at full reach is about 30 mm at
// the tool. That is far too coarse to place a box with, and it does not need
// to be: this plans the TRAVERSE between poses. The final approach is still
// the sequence's slow force guided descent, which is accurate to a fraction
// of a millimetre and does not care how the arm arrived.
#pragma once

#include "est/arm_model.hpp"
#include "task/survey.hpp"
#include "types.hpp"

namespace ppa {

// 96 cells per joint over the full turn, so 3.75 degrees each. The whole grid
// is 9216 cells: a 1.2 KB bitmap for what is blocked and 18 KB of distance
// field, which fits an STM32F407's 192 KB with room to spare.
constexpr int kGridN = 96;
constexpr int kGridCells = kGridN * kGridN;
constexpr float kGridStepRad = 6.28318530718f / static_cast<float>(kGridN);

constexpr int kMaxPath = 64;

// What the arm must not pass through. Boxes come from the survey, so the
// planner avoids what the arm actually saw rather than what it was told.
struct Scene {
    float table_y{-0.150f};
    // Clearance kept from every surface. The arm is a line in this model
    // while a real one has thickness, and the grid is coarse, so a margin
    // covers both.
    float margin_m{0.012f};

    int n_boxes{0};
    float box_x[kMaxObjects]{};
    float box_top_y[kMaxObjects]{};
    float box_half_w[kMaxObjects]{};

    // The box currently being picked must be ignored, or the goal pose is
    // inside an obstacle and no plan exists.
    int ignore_box{-1};

    void from_survey(const Survey& s, float table);
};

struct PlanResult {
    bool found{false};
    int n{0};
    float q1[kMaxPath]{};
    float q2[kMaxPath]{};
    // Diagnostics worth having: a plan that failed because the goal was
    // blocked is a different problem from one that failed because the arm is
    // walled in.
    bool start_blocked{false};
    bool goal_blocked{false};
    int cells_blocked{0};
    int cells_searched{0};
};

class Planner {
public:
    // Marks every cell that would put the arm through something. Done once
    // per scene, not once per plan.
    void build(const Scene& scene);

    // Route from one pose to another through the free cells.
    bool plan(float from_q1, float from_q2, float to_q1, float to_q2,
              PlanResult* out);

    bool blocked_at(float q1, float q2) const;
    int blocked_count() const { return blocked_count_; }

    // True if the straight line between two poses is clear, which is the
    // common case and worth answering without a search.
    bool direct_clear(float from_q1, float from_q2, float to_q1, float to_q2) const;

private:
    static int bin(float q);
    static float centre(int i);
    bool pose_collides(const Scene& s, float q1, float q2) const;

    // One bit per cell.
    u32 blocked_[(kGridCells + 31) / 32]{};
    u16 dist_[kGridCells]{};
    u16 queue_[kGridCells]{};
    int blocked_count_{0};
    bool built_{false};
};

}  // namespace ppa
