// Fixed size linear algebra, sized at compile time.
//
// The EKF needs matrix products, transposes and one small inverse. On a
// microcontroller none of that may touch the heap, and the dimensions are all
// known when the filter is written, so every matrix here carries its shape in
// the type. A dimension mistake becomes a compile error instead of a runtime
// crash, and the whole filter fits in static storage.
//
// Storage is row major. Indexing is (row, col), zero based.
#pragma once

#include <cmath>
#include <cstdlib>

#include "types.hpp"

namespace ppa {

template <int R, int C>
struct Mat {
    static constexpr int kRows = R;
    static constexpr int kCols = C;

    float d[R * C]{};

    float& operator()(int r, int c) { return d[r * C + c]; }
    const float& operator()(int r, int c) const { return d[r * C + c]; }

    // Single index access, for vectors.
    float& operator[](int i) { return d[i]; }
    const float& operator[](int i) const { return d[i]; }

    static Mat zero() { return Mat{}; }

    static Mat identity() {
        static_assert(R == C, "identity needs a square shape");
        Mat m{};
        for (int i = 0; i < R; ++i) m(i, i) = 1.0f;
        return m;
    }

    // Diagonal matrix from a list of variances, the usual way noise
    // covariances get written down.
    static Mat diag(const float (&v)[R]) {
        static_assert(R == C, "diag needs a square shape");
        Mat m{};
        for (int i = 0; i < R; ++i) m(i, i) = v[i];
        return m;
    }
};

template <int N>
using Vec = Mat<N, 1>;

template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> out{};
    for (int i = 0; i < R; ++i) {
        for (int k = 0; k < K; ++k) {
            const float aik = a(i, k);
            if (aik == 0.0f) continue;  // sparse Jacobians are the common case
            for (int j = 0; j < C; ++j) out(i, j) += aik * b(k, j);
        }
    }
    return out;
}

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> out{};
    for (int i = 0; i < R * C; ++i) out.d[i] = a.d[i] + b.d[i];
    return out;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> out{};
    for (int i = 0; i < R * C; ++i) out.d[i] = a.d[i] - b.d[i];
    return out;
}

template <int R, int C>
Mat<R, C> operator*(const Mat<R, C>& a, float s) {
    Mat<R, C> out{};
    for (int i = 0; i < R * C; ++i) out.d[i] = a.d[i] * s;
    return out;
}

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& a) {
    Mat<C, R> out{};
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) out(j, i) = a(i, j);
    return out;
}

template <int N>
float trace(const Mat<N, N>& a) {
    float t = 0.0f;
    for (int i = 0; i < N; ++i) t += a(i, i);
    return t;
}

// Force exact symmetry. Repeated EKF updates accumulate asymmetry in the last
// bits of the covariance, and once P stops being symmetric the innovation
// covariance can go indefinite and the filter falls over. Cheap insurance,
// applied after every update.
template <int N>
Mat<N, N> symmetrise(const Mat<N, N>& a) {
    Mat<N, N> out{};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) out(i, j) = 0.5f * (a(i, j) + a(j, i));
    return out;
}

// Inverse by Gauss Jordan with partial pivoting. Only ever called on the
// innovation covariance, which is at most 2x2 in this filter, so the cubic
// cost is irrelevant and the clarity is worth more than a Cholesky solve.
//
// Returns false and leaves out untouched if the matrix is singular, which the
// caller must treat as a rejected measurement rather than ignore.
template <int N>
bool invert(const Mat<N, N>& a, Mat<N, N>* out) {
    float m[N][2 * N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) m[i][j] = a(i, j);
        for (int j = 0; j < N; ++j) m[i][N + j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int col = 0; col < N; ++col) {
        int piv = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(m[r][col]) > std::fabs(m[piv][col])) piv = r;

        if (std::fabs(m[piv][col]) < 1e-12f) return false;

        if (piv != col)
            for (int j = 0; j < 2 * N; ++j) {
                const float t = m[col][j];
                m[col][j] = m[piv][j];
                m[piv][j] = t;
            }

        const float inv = 1.0f / m[col][col];
        for (int j = 0; j < 2 * N; ++j) m[col][j] *= inv;

        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            const float f = m[r][col];
            if (f == 0.0f) continue;
            for (int j = 0; j < 2 * N; ++j) m[r][j] -= f * m[col][j];
        }
    }

    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) (*out)(i, j) = m[i][N + j];
    return true;
}

// Wrap to (-pi, pi]. Every angle difference in the filter goes through this.
// Getting it wrong is the classic way a heading estimate slams 360 degrees.
//
// Written with fmod rather than a subtraction loop, and that is not a
// micro-optimisation. The loop version was:
//
//     while (a > kPi) a -= kTwoPi;
//
// which never terminates for a = +infinity, because infinity minus two pi is
// still infinity. A control instability produced one, and the whole firmware
// hung inside this four line function: no crash, no error, just a process
// that stopped responding. That is the worst possible failure mode in
// something that is supposed to be running a safety loop at 1 kHz.
//
// So: bounded work for every input, and a defined answer for the ones that
// should never happen. A non-finite angle is returned as zero rather than
// propagated, because the callers all have validity checks of their own and
// a quiet zero is far easier to notice than a hang.
inline float wrap_pi(float a) {
    constexpr float kPi = 3.14159265358979f;
    constexpr float kTwoPi = 2.0f * kPi;
    if (!std::isfinite(a)) return 0.0f;
    float r = std::fmod(a + kPi, kTwoPi);
    if (r < 0.0f) r += kTwoPi;
    return r - kPi;
}

}  // namespace ppa
