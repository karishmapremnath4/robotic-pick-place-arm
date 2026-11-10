#include "task/planner.hpp"

#include <cmath>

namespace ppa {

namespace {

constexpr u16 kUnreached = 0xFFFF;

// Points sampled along each link when testing it against a surface.
//
// The links are tested as sampled points rather than as exact segments. An
// exact segment against rectangle test is not hard, but at this grid
// resolution it buys nothing: neighbouring cells differ by 3.75 degrees,
// which moves the tool about 30 mm, so an obstacle thinner than the sampling
// gap is already finer than the grid can represent. The clearance margin in
// Scene covers the difference, and it is stated rather than hoped.
constexpr int kLinkSamples = 10;

bool point_in_box(float x, float y, float cx, float half_w, float top_y,
                  float table_y, float margin) {
    return x > cx - half_w - margin && x < cx + half_w + margin &&
           y < top_y + margin && y > table_y - margin;
}

}  // namespace

void Scene::from_survey(const Survey& s, float table) {
    table_y = table;
    n_boxes = 0;
    for (int i = 0; i < s.object_count() && n_boxes < kMaxObjects; ++i) {
        const DetectedObject& o = s.object(i);
        box_x[n_boxes] = o.centre_x;
        box_top_y[n_boxes] = o.top_y;
        box_half_w[n_boxes] = 0.5f * o.width_m;
        ++n_boxes;
    }
}

int Planner::bin(float q) {
    const float w = wrap_pi(q);
    int i = static_cast<int>((w + 3.14159265358979f) / kGridStepRad);
    if (i < 0) i = 0;
    if (i >= kGridN) i = kGridN - 1;
    return i;
}

float Planner::centre(int i) {
    return -3.14159265358979f + (static_cast<float>(i) + 0.5f) * kGridStepRad;
}

bool Planner::pose_collides(const Scene& s, float q1, float q2) const {
    const float c1 = std::cos(q1), s1 = std::sin(q1);
    const float c12 = std::cos(q1 + q2), s12 = std::sin(q1 + q2);

    const float ex = kLink1M * c1, ey = kLink1M * s1;
    const float tx = ex + kLink2M * c12, ty = ey + kLink2M * s12;

    for (int link = 0; link < 2; ++link) {
        const float x0 = (link == 0) ? 0.0f : ex;
        const float y0 = (link == 0) ? 0.0f : ey;
        const float x1 = (link == 0) ? ex : tx;
        const float y1 = (link == 0) ? ey : ty;

        for (int k = 0; k <= kLinkSamples; ++k) {
            const float u = static_cast<float>(k) / kLinkSamples;
            const float x = x0 + u * (x1 - x0);
            const float y = y0 + u * (y1 - y0);

            // Below the table is through the table.
            if (y < s.table_y + s.margin_m) return true;

            for (int b = 0; b < s.n_boxes; ++b) {
                if (b == s.ignore_box) continue;
                if (point_in_box(x, y, s.box_x[b], s.box_half_w[b],
                                 s.box_top_y[b], s.table_y, s.margin_m))
                    return true;
            }
        }
    }
    return false;
}

void Planner::build(const Scene& scene) {
    for (int i = 0; i < static_cast<int>(sizeof(blocked_) / sizeof(blocked_[0])); ++i)
        blocked_[i] = 0;
    blocked_count_ = 0;

    for (int i = 0; i < kGridN; ++i) {
        const float q1 = centre(i);
        for (int j = 0; j < kGridN; ++j) {
            const float q2 = centre(j);
            if (pose_collides(scene, q1, q2)) {
                const int cell = i * kGridN + j;
                blocked_[cell >> 5] |= (1u << (cell & 31));
                ++blocked_count_;
            }
        }
    }
    built_ = true;
}

bool Planner::blocked_at(float q1, float q2) const {
    const int cell = bin(q1) * kGridN + bin(q2);
    return (blocked_[cell >> 5] >> (cell & 31)) & 1u;
}

bool Planner::direct_clear(float from_q1, float from_q2, float to_q1,
                           float to_q2) const {
    // Sampled along the straight line in joint space, which is what the
    // position loops actually follow when handed a single target.
    const float d1 = wrap_pi(to_q1 - from_q1);
    const float d2 = wrap_pi(to_q2 - from_q2);
    const float span = std::fmax(std::fabs(d1), std::fabs(d2));
    const int steps = static_cast<int>(span / (0.5f * kGridStepRad)) + 2;
    for (int k = 0; k <= steps; ++k) {
        const float u = static_cast<float>(k) / steps;
        if (blocked_at(from_q1 + u * d1, from_q2 + u * d2)) return false;
    }
    return true;
}

bool Planner::plan(float from_q1, float from_q2, float to_q1, float to_q2,
                   PlanResult* out) {
    *out = PlanResult{};
    out->cells_blocked = blocked_count_;
    if (!built_) return false;

    const int si = bin(from_q1), sj = bin(from_q2);
    const int gi = bin(to_q1), gj = bin(to_q2);
    const int start = si * kGridN + sj;
    const int goal = gi * kGridN + gj;

    out->start_blocked = blocked_at(from_q1, from_q2);
    out->goal_blocked = blocked_at(to_q1, to_q2);

    // A blocked start is survivable: the arm is already there, so the first
    // move has to be allowed to leave. A blocked goal is not, because there
    // is nowhere to arrive.
    if (out->goal_blocked) return false;

    for (int i = 0; i < kGridCells; ++i) dist_[i] = kUnreached;

    // Breadth first from the GOAL, giving a distance field over the whole
    // free space, then walk downhill from the start. Searching from the goal
    // rather than the start means the field can be reused for several starts,
    // and it makes the descent a simple local choice with no backtracking.
    //
    // Uniform cost, eight connected. Diagonals are treated as costing the
    // same as orthogonals, which slightly overvalues them and makes paths a
    // little more diagonal than optimal. That is a worse path, not a wrong
    // one, and it keeps the search to a plain FIFO with no priority queue
    // and no dynamic memory.
    int head = 0, tail = 0;
    dist_[goal] = 0;
    queue_[tail++] = static_cast<u16>(goal);

    while (head < tail) {
        const int cell = queue_[head++];
        ++out->cells_searched;
        if (cell == start) break;

        const int ci = cell / kGridN, cj = cell % kGridN;
        const u16 d = static_cast<u16>(dist_[cell] + 1);

        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                if (di == 0 && dj == 0) continue;
                // Both joints wrap, so the grid is a torus and the edges
                // join up. That is not a trick: the arm really can reach the
                // same pose by turning the other way.
                const int ni = (ci + di + kGridN) % kGridN;
                const int nj = (cj + dj + kGridN) % kGridN;
                const int n = ni * kGridN + nj;
                if (dist_[n] != kUnreached) continue;
                if ((blocked_[n >> 5] >> (n & 31)) & 1u) continue;
                dist_[n] = d;
                queue_[tail++] = static_cast<u16>(n);
            }
        }
    }

    if (dist_[start] == kUnreached) return false;

    // Walk downhill to the goal.
    int cell = start;
    out->n = 0;
    while (out->n < kMaxPath) {
        const int ci = cell / kGridN, cj = cell % kGridN;
        out->q1[out->n] = centre(ci);
        out->q2[out->n] = centre(cj);
        ++out->n;
        if (cell == goal) break;

        int best = -1;
        u16 best_d = dist_[cell];
        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                if (di == 0 && dj == 0) continue;
                const int ni = (ci + di + kGridN) % kGridN;
                const int nj = (cj + dj + kGridN) % kGridN;
                const int n = ni * kGridN + nj;
                if (dist_[n] == kUnreached) continue;
                if (dist_[n] < best_d) { best_d = dist_[n]; best = n; }
            }
        }
        if (best < 0) return false;  // stuck, which should be impossible
        cell = best;
    }

    // Finish exactly on the requested pose rather than on a cell centre, so
    // the 3.75 degree grid does not become a 3.75 degree positioning error.
    if (out->n < kMaxPath) {
        out->q1[out->n] = wrap_pi(to_q1);
        out->q2[out->n] = wrap_pi(to_q2);
        ++out->n;
    } else {
        out->q1[kMaxPath - 1] = wrap_pi(to_q1);
        out->q2[kMaxPath - 1] = wrap_pi(to_q2);
    }

    out->found = true;
    return true;
}

}  // namespace ppa
