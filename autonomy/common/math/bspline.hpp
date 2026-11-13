/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file bspline.hpp
 * @brief Clamped cubic B-splines (Cox–de Boor) on scalars or Eigen points.
 *
 * Knots are uniform and clamped: the first and last four knots are 0 and 1,
 * so the curve interpolates the first and last control points. With exactly
 * four control points the knot vector is [0,0,0,0,1,1,1,1] and the curve is
 * the same cubic Bézier as @ref CubicBezier.
 *
 * This is not a Bézier corridor piece. SANDO's four-handle cubic stays in
 * bezier.hpp. SanD-Planner predicts N (typically 8 or 12) B-spline control
 * points and should use @ref ClampedCubicBSpline.
 *
 * Point must support addition, subtraction, scalar multiplication, and for
 * arc-length resampling a Euclidean norm (Eigen vectors) or std::abs (scalars).
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy {
namespace common {
namespace math {
namespace {

template <typename Point>
Point BSplineZero() {
    if constexpr (std::is_arithmetic_v<Point>) {
        return Point(0);
    } else {
        return Point::Zero();
    }
}

template <typename Point>
double PointSeparation(const Point& from, const Point& to) {
    if constexpr (std::is_arithmetic_v<Point>) {
        return std::abs(to - from);
    } else {
        return (to - from).norm();
    }
}

inline std::vector<double> MakeClampedUniformKnots(int control_point_count,
                                                   int degree) {
    const int knot_count = control_point_count + degree + 1;
    std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);
    for (int i = 0; i <= degree; ++i) {
        knots[static_cast<std::size_t>(i)] = 0.0;
        knots[static_cast<std::size_t>(knot_count - 1 - i)] = 1.0;
    }
    const int interior = control_point_count - degree - 1;
    for (int i = 1; i <= interior; ++i) {
        knots[static_cast<std::size_t>(degree + i)] =
            static_cast<double>(i) / static_cast<double>(interior + 1);
    }
    return knots;
}

inline int FindKnotSpan(const std::vector<double>& knots, int control_point_count,
                        int degree, double parameter) {
    const double last = knots[static_cast<std::size_t>(control_point_count)];
    if (parameter >= last) {
        return control_point_count - 1;
    }
    int low = degree;
    int high = control_point_count;
    int mid = (low + high) / 2;
    while (parameter < knots[static_cast<std::size_t>(mid)] ||
           parameter >= knots[static_cast<std::size_t>(mid + 1)]) {
        if (parameter < knots[static_cast<std::size_t>(mid)]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

}  // namespace

/**
 * @brief Clamped cubic B-spline with a cached uniform knot vector.
 *
 * Knots are built once in the constructor so repeated @ref Evaluate calls
 * (sampling a SanD candidate) do not rebuild the knot vector.
 */
template <typename Point>
class ClampedCubicBSpline {
public:
    /**
     * @brief Polynomial degree (always 3).
     */
    static constexpr int kDegree = 3;

    ClampedCubicBSpline() = default;

    /**
     * @brief Build from control points in evaluation order.
     *
     * @param[in] control_points N points. N >= 4 uses de Boor; 1 <= N < 4
     *                           falls back to linear interpolation of the
     *                           endpoints; N = 0 evaluates to zero.
     */
    explicit ClampedCubicBSpline(std::vector<Point> control_points)
        : control_points_(std::move(control_points)) {
        const int count = static_cast<int>(control_points_.size());
        if (count >= kDegree + 1) {
            knots_ = MakeClampedUniformKnots(count, kDegree);
        }
    }

    /**
     * @brief Number of control points.
     */
    int control_point_count() const {
        return static_cast<int>(control_points_.size());
    }

    /**
     * @brief Control points (const).
     */
    const std::vector<Point>& control_points() const { return control_points_; }

    /**
     * @brief Cached knot vector; empty when there are fewer than four points.
     */
    const std::vector<double>& knots() const { return knots_; }

    /**
     * @brief First control point, or zero when empty.
     */
    Point start() const {
        if (control_points_.empty()) {
            return BSplineZero<Point>();
        }
        return control_points_.front();
    }

    /**
     * @brief Last control point, or zero when empty.
     */
    Point end() const {
        if (control_points_.empty()) {
            return BSplineZero<Point>();
        }
        return control_points_.back();
    }

    /**
     * @brief Curve point at @p parameter, clamped to [0, 1].
     *
     * @param[in] parameter Normalized parameter.
     */
    Point Evaluate(double parameter) const {
        return EvaluateDerivative(0, parameter);
    }

    /**
     * @brief Derivative of order @p order with respect to the unit parameter.
     *
     * Order 0 is position. Higher orders use repeated B-spline differentiation
     * (scaled consecutive differences of control points) then de Boor. Order
     * greater than 3 is identically zero for a cubic.
     *
     * @param[in] order Derivative order, 0 through 3.
     * @param[in] parameter Normalized parameter; clamped to [0, 1].
     */
    Point EvaluateDerivative(int order, double parameter) const {
        if (order < 0 || control_points_.empty()) {
            return BSplineZero<Point>();
        }
        const double u = std::min(1.0, std::max(0.0, parameter));
        const int count = static_cast<int>(control_points_.size());
        if (count == 1) {
            return order == 0 ? control_points_.front() : BSplineZero<Point>();
        }
        if (count < kDegree + 1) {
            if (order == 0) {
                return control_points_.front() * (1.0 - u) +
                       control_points_.back() * u;
            }
            if (order == 1) {
                return control_points_.back() - control_points_.front();
            }
            return BSplineZero<Point>();
        }
        if (order > kDegree) {
            return BSplineZero<Point>();
        }
        if (order == 0) {
            return DeBoor(control_points_, knots_, kDegree, u);
        }

        std::vector<Point> points = control_points_;
        std::vector<double> knots = knots_;
        int degree = kDegree;
        for (int step = 0; step < order; ++step) {
            const int n = static_cast<int>(points.size());
            std::vector<Point> next(static_cast<std::size_t>(n - 1));
            for (int i = 0; i < n - 1; ++i) {
                const double denom =
                    knots[static_cast<std::size_t>(i + degree + 1)] -
                    knots[static_cast<std::size_t>(i + 1)];
                const Point delta =
                    points[static_cast<std::size_t>(i + 1)] -
                    points[static_cast<std::size_t>(i)];
                next[static_cast<std::size_t>(i)] =
                    denom > 1e-12 ? delta * (static_cast<double>(degree) / denom)
                                  : BSplineZero<Point>();
            }
            points = std::move(next);
            knots.erase(knots.begin());
            knots.pop_back();
            --degree;
        }
        return DeBoor(points, knots, degree, u);
    }

    /**
     * @brief Uniform samples from parameter 0 to 1, inclusive.
     *
     * @param[in] sample_count Number of samples; values below 2 become 2.
     */
    std::vector<Point> Sample(int sample_count) const {
        const int count = std::max(2, sample_count);
        std::vector<Point> samples(static_cast<std::size_t>(count),
                                   BSplineZero<Point>());
        const double denom = static_cast<double>(count - 1);
        for (int i = 0; i < count; ++i) {
            samples[static_cast<std::size_t>(i)] =
                Evaluate(static_cast<double>(i) / denom);
        }
        return samples;
    }

private:
    static Point DeBoor(const std::vector<Point>& points,
                        const std::vector<double>& knots, int degree,
                        double parameter) {
        const int n = static_cast<int>(points.size());
        if (n <= 0) {
            return BSplineZero<Point>();
        }
        const int safe_degree = std::max(0, degree);
        const int span = FindKnotSpan(knots, n, safe_degree, parameter);
        if (safe_degree <= 0) {
            return points[static_cast<std::size_t>(
                std::min(n - 1, std::max(0, span)))];
        }
        std::array<Point, kDegree + 1> work;
        for (int j = 0; j <= degree; ++j) {
            const int index = span - degree + j;
            work[static_cast<std::size_t>(j)] =
                points[static_cast<std::size_t>(std::min(n - 1, std::max(0, index)))];
        }
        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                const int left = span - degree + j;
                const double denom =
                    knots[static_cast<std::size_t>(left + degree - r + 1)] -
                    knots[static_cast<std::size_t>(left)];
                const double alpha =
                    denom > 1e-12
                        ? (parameter - knots[static_cast<std::size_t>(left)]) /
                              denom
                        : 0.0;
                work[static_cast<std::size_t>(j)] =
                    work[static_cast<std::size_t>(j - 1)] * (1.0 - alpha) +
                    work[static_cast<std::size_t>(j)] * alpha;
            }
        }
        return work[static_cast<std::size_t>(degree)];
    }

    std::vector<Point> control_points_;
    std::vector<double> knots_;
};

/**
 * @brief Resample a polyline to @p sample_count points of equal arc length.
 *
 * @param[in] points Polyline in Point-space.
 * @param[in] sample_count Output count; values below 2 become 2.
 * @return Equally spaced samples; empty when @p points is empty.
 */
template <typename Point>
std::vector<Point> ResamplePolylineArcLength(const std::vector<Point>& points,
                                             int sample_count) {
    if (points.empty()) {
        return {};
    }
    const int count = std::max(2, sample_count);
    if (points.size() == 1) {
        return std::vector<Point>(static_cast<std::size_t>(count), points.front());
    }
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] =
            cumulative[i - 1] + PointSeparation(points[i - 1], points[i]);
    }
    const double length = cumulative.back();
    if (length < 1e-9) {
        return std::vector<Point>(static_cast<std::size_t>(count), points.front());
    }
    std::vector<Point> out(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double s =
            length * static_cast<double>(i) / static_cast<double>(count - 1);
        auto it = std::lower_bound(cumulative.begin(), cumulative.end(), s);
        std::size_t index = static_cast<std::size_t>(
            std::distance(cumulative.begin(), it));
        if (index == 0) {
            out[static_cast<std::size_t>(i)] = points.front();
            continue;
        }
        if (index >= points.size()) {
            out[static_cast<std::size_t>(i)] = points.back();
            continue;
        }
        const double span = cumulative[index] - cumulative[index - 1];
        const double alpha =
            span > 1e-12 ? (s - cumulative[index - 1]) / span : 0.0;
        out[static_cast<std::size_t>(i)] =
            points[index - 1] * (1.0 - alpha) + points[index] * alpha;
    }
    return out;
}

}  // namespace math
}  // namespace common
}  // namespace autonomy
