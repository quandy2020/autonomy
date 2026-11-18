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
 * Knots are uniform by default, with an explicit nonuniform overload. Both
 * constructors clamp the first and last four knots to 0 and 1,
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
#include <stdexcept>
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
 * @brief Clamped cubic B-spline with cached knots and derivative curves.
 *
 * Knots and derivative curves of orders 1..3 are built once in the constructor.
 * Repeated @ref Evaluate and @ref EvaluateDerivative calls do not allocate
 * spline storage. Point algebra may still allocate for dynamic point types.
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
            CacheDerivatives();
        }
    }

    /**
     * @brief Build a cubic with explicit normalized, clamped knots.
     * @param control_points At least four controls in evaluation order.
     * @param knots N+4 knots with simple interior spans; endpoints repeat four times.
     * @throws std::invalid_argument For an invalid size, endpoint or interior span.
     * This overload preserves nonuniform physical-reference geometry while
     * reusing the same evaluator as the uniform SanD constructor.
     */
    ClampedCubicBSpline(std::vector<Point> control_points, std::vector<double> knots)
        : control_points_(std::move(control_points)), knots_(std::move(knots)) {
        const auto count = control_points_.size();
        if (count < 4 || knots_.size() != count + 4) {
            throw std::invalid_argument("cubic spline requires N>=4 and N+4 knots");
        }
        for (std::size_t i = 0; i < 4; ++i) {
            if (knots_[i] != 0.0 || knots_[count + i] != 1.0) {
                throw std::invalid_argument("cubic endpoints must be clamped at 0 and 1");
            }
        }
        for (std::size_t i = 4; i <= count; ++i) {
            if (!std::isfinite(knots_[i]) || knots_[i] - knots_[i - 1] < 1e-9) {
                throw std::invalid_argument("interior cubic knots need simple positive spans");
            }
        }
        CacheDerivatives();
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
     * @brief Solve the first three controls for a prescribed C2 start state.
     * @param position Position at normalized parameter u=0.
     * @param first_derivative First derivative with respect to u, not seconds.
     * @param second_derivative Second derivative with respect to u.
     * @return Replacement controls Q0, Q1 and Q2, using this curve's actual knots.
     * @throws std::invalid_argument For a short curve or nonfinite boundary data.
     * @throws std::overflow_error If the replacement controls are nonfinite.
     * @details The curve is unchanged. Copy these three controls into a new
     * curve while preserving the remaining controls and knots. Physical-time
     * callers multiply velocity by duration and acceleration by duration squared
     * before calling. Point must be arithmetic or provide allFinite(). This
     * operation neither matches jerk nor certifies dynamic or collision limits.
     */
    std::array<Point, 3> StartControlPoints(const Point& position,
                                           const Point& first_derivative,
                                           const Point& second_derivative) const {
        if (control_points_.size() < kDegree + 1) {
            throw std::invalid_argument("C2 start state requires a cubic spline");
        }
        const auto finite = [](const Point& point) {
            if constexpr (std::is_arithmetic_v<Point>) {
                return std::isfinite(point);
            } else {
                return point.allFinite();
            }
        };
        if (!finite(position) || !finite(first_derivative) || !finite(second_derivative)) {
            throw std::invalid_argument("nonfinite cubic start state");
        }
        const double first_span = knots_[4] - knots_[1];
        const double second_span = knots_[5] - knots_[2];
        const double acceleration_span = knots_[4] - knots_[2];
        const Point next_derivative =
            first_derivative + second_derivative * (acceleration_span / 2.0);
        std::array<Point, 3> controls;
        controls[0] = position;
        controls[1] = position + first_derivative * (first_span / 3.0);
        controls[2] = controls[1] + next_derivative * (second_span / 3.0);
        for (const auto& control : controls) {
            if (!finite(control)) throw std::overflow_error("cubic start controls overflow");
        }
        return controls;
    }

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

        const auto index = static_cast<std::size_t>(order - 1);
        return DeBoor(derivative_controls_[index], derivative_knots_[index],
                      kDegree - order, u);
    }

    /**
     * @brief Controls of the derivative with respect to normalized u.
     * @param order Derivative order, 0 through 3; out of range returns no controls.
     * @return Controls whose convex hull bounds the derivative; physical-time
     * callers divide by duration^order. The result is an owned copy, so changing
     * it does not modify subsequent evaluations. Short curves retain the linear
     * fallback.
     */
    std::vector<Point> DerivativeControlPoints(int order) const {
        if (order < 0 || order > kDegree || control_points_.empty()) return {};
        if (order == 0) return control_points_;
        if (control_points_.size() < 4) {
            return {order == 1 ? Point(end() - start()) : BSplineZero<Point>()};
        }
        return derivative_controls_[static_cast<std::size_t>(order - 1)];
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
    /**
     * @brief Build immutable derivative storage after cubic knots are validated.
     * @details Each derivative reduces the degree and removes an endpoint knot
     * pair. The original knots may be nonuniform. Construction is the only
     * place that allocates this storage; const evaluation only reads it.
     * @pre At least four controls and a valid normalized clamped knot vector.
     */
    void CacheDerivatives() {
        for (int order = 1; order <= kDegree; ++order) {
            const auto index = static_cast<std::size_t>(order - 1);
            const auto& points = order == 1 ? control_points_
                                            : derivative_controls_[index - 1];
            const auto& knots = order == 1 ? knots_ : derivative_knots_[index - 1];
            const int degree = kDegree - order + 1;
            auto& next = derivative_controls_[index];
            next.resize(points.size() - 1);
            for (std::size_t i = 0; i < next.size(); ++i) {
                const double denom = knots[i + static_cast<std::size_t>(degree) + 1] -
                                     knots[i + 1];
                const Point delta = points[i + 1] - points[i];
                next[i] = denom > 1e-12
                    ? delta * (static_cast<double>(degree) / denom)
                    : BSplineZero<Point>();
            }
            derivative_knots_[index].assign(knots.begin() + 1, knots.end() - 1);
        }
    }

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

    std::vector<Point> control_points_;  ///< Curve controls in evaluation order.
    std::vector<double> knots_;          ///< Cached normalized, clamped knot vector.
    std::array<std::vector<Point>, kDegree> derivative_controls_;  ///< Cached controls for orders 1..3.
    std::array<std::vector<double>, kDegree> derivative_knots_;  ///< Cached reduced knot vectors for orders 1..3.
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
