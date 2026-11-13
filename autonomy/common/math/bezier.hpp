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
 * @file bezier.hpp
 * @brief Cubic Bézier curves (de Casteljau) on scalars or Eigen points.
 *
 * One cubic piece has four control points P0..P3 and parameter u in [0, 1].
 * The curve interpolates P0 and P3; P1 and P2 shape the interior. The image
 * of the curve lies in the convex hull of the control points, which is the
 * property SANDO uses to turn corridor constraints into finitely many
 * inequalities.
 *
 * This is not a B-spline. Clamped cubic B-splines live in bspline.hpp;
 * do not feed N>4 SanD control points into CubicBezier.
 *
 * Point must support addition, subtraction, and scalar multiplication
 * (double and Eigen::Vector2d / Vector3d are the intended types).
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace autonomy {
namespace common {
namespace math {

/**
 * @brief Additive identity for a Bézier point type.
 *
 * Eigen default constructors leave coefficients uninitialized; this helper
 * returns a true zero so unused derivative orders stay defined.
 *
 * @tparam Point Scalar or Eigen column vector with a Zero() factory.
 */
template <typename Point>
inline Point ZeroPoint() {
    if constexpr (std::is_arithmetic_v<Point>) {
        return Point(0);
    } else {
        return Point::Zero();
    }
}

/**
 * @brief Affine combination (1 - weight) * from + weight * to.
 *
 * @param[in] from Value at weight 0.
 * @param[in] to Value at weight 1.
 * @param[in] weight Blend in [0, 1] (not clamped).
 */
template <typename Point>
inline Point MixPoints(const Point& from, const Point& to, double weight) {
    return from + (to - from) * weight;
}

/**
 * @brief One cubic Bézier segment in Point-space.
 *
 * Evaluation uses de Casteljau (numerically stabler than expanding Bernstein
 * polynomials). Derivatives follow the standard degree-reduction: the first
 * derivative is a quadratic Bézier on 3 (P_{i+1} - P_i), and so on.
 */
template <typename Point>
class CubicBezier {
public:
    /**
     * @brief Polynomial degree (always 3).
     */
    static constexpr int kDegree = 3;

    /**
     * @brief Number of geometric control points (always 4).
     */
    static constexpr int kControlPointCount = 4;

    CubicBezier()
        : control_points_{ZeroPoint<Point>(), ZeroPoint<Point>(),
                          ZeroPoint<Point>(), ZeroPoint<Point>()} {}

    /**
     * @brief Construct from P0, P1, P2, P3 in that order.
     *
     * @param[in] control_points Four points; index 0 is the start, 3 the end.
     */
    explicit CubicBezier(std::array<Point, kControlPointCount> control_points)
        : control_points_(std::move(control_points)) {}

    /**
     * @brief Named constructor from four control points.
     */
    static CubicBezier FromControlPoints(const Point& start,
                                         const Point& control_1,
                                         const Point& control_2,
                                         const Point& end) {
        return CubicBezier({start, control_1, control_2, end});
    }

    /**
     * @brief Build the unique cubic Bézier of a monomial p(u) = c0 + c1 u + c2 u² + c3 u³.
     *
     * Coefficient layout matches SANDO: power 0 at index 0. Parameter u is
     * assumed to live in [0, 1].
     *
     * @param[in] coefficients Monomial coefficients, lowest power first.
     */
    static CubicBezier FromMonomial(
        const std::array<Point, kControlPointCount>& coefficients) {
        const Point& c0 = coefficients[0];
        const Point& c1 = coefficients[1];
        const Point& c2 = coefficients[2];
        const Point& c3 = coefficients[3];
        const Point p0 = c0;
        const Point p1 = c0 + c1 * (1.0 / 3.0);
        const Point p2 = c0 + c1 * (2.0 / 3.0) + c2 * (1.0 / 3.0);
        const Point p3 = c0 + c1 + c2 + c3;
        return CubicBezier({p0, p1, p2, p3});
    }

    /**
     * @brief Inverse of @ref FromMonomial: c0 + c1 u + c2 u² + c3 u³, u in [0, 1].
     *
     * @return Coefficients, lowest power first.
     */
    std::array<Point, kControlPointCount> ToMonomial() const {
        const Point& p0 = control_points_[0];
        const Point& p1 = control_points_[1];
        const Point& p2 = control_points_[2];
        const Point& p3 = control_points_[3];
        std::array<Point, kControlPointCount> coefficients;
        coefficients[0] = p0;
        coefficients[1] = (p1 - p0) * 3.0;
        coefficients[2] = (p0 - p1 * 2.0 + p2) * 3.0;
        coefficients[3] = (p3 - p0) + (p1 - p2) * 3.0;
        return coefficients;
    }

    /**
     * @brief Control points P0..P3 (const).
     */
    const std::array<Point, kControlPointCount>& control_points() const {
        return control_points_;
    }

    /**
     * @brief Control points P0..P3 (mutable).
     */
    std::array<Point, kControlPointCount>& control_points() {
        return control_points_;
    }

    /**
     * @brief Start of the curve (P0).
     */
    const Point& start() const { return control_points_[0]; }

    /**
     * @brief End of the curve (P3).
     */
    const Point& end() const { return control_points_[3]; }

    /**
     * @brief Position at @p parameter, clamped to [0, 1].
     *
     * @param[in] parameter Curve parameter (values outside [0, 1] are clamped).
     * @return Point on the curve.
     */
    Point Evaluate(double parameter) const {
        return EvaluateDerivative(0, parameter);
    }

    /**
     * @brief Derivative of order @p order with respect to the unit parameter u.
     *
     * Order 0 is position. Order 1 is dB/du (not physical velocity). Order 2
     * is d²B/du². Order 3 is the constant jerk in u. Order greater than 3 is
     * identically zero for a cubic. Use @ref EvaluateWithDuration to convert
     * to time derivatives when the piece lasts @c duration seconds.
     *
     * @param[in] order Derivative order, 0 through 3.
     * @param[in] parameter Curve parameter; clamped to [0, 1].
     * @return Derivative in Point units per u^order.
     */
    Point EvaluateDerivative(int order, double parameter) const {
        if (order < 0) {
            return ZeroPoint<Point>();
        }
        if (order > kDegree) {
            return ZeroPoint<Point>();
        }
        const double u = ClampUnit(parameter);
        std::array<Point, kControlPointCount> points = control_points_;
        int remaining_degree = kDegree;
        double scale = 1.0;
        for (int step = 0; step < order; ++step) {
            scale *= static_cast<double>(remaining_degree);
            for (int i = 0; i < remaining_degree; ++i) {
                points[static_cast<std::size_t>(i)] =
                    points[static_cast<std::size_t>(i + 1)] -
                    points[static_cast<std::size_t>(i)];
            }
            --remaining_degree;
        }
        Point value = DeCasteljau(points, remaining_degree, u);
        return value * scale;
    }

    /**
     * @brief Time-scaled derivative when the piece occupies @p duration seconds.
     *
     * Maps t in [0, duration] to u = t / duration, then divides the u-derivative
     * of order k by duration^k. Matches SANDO's physical velocity / acceleration.
     *
     * @param[in] order Derivative order, 0 through 3.
     * @param[in] time Seconds from the start of the piece; clamped to [0, duration].
     * @param[in] duration Piece length in seconds. Values below 1e-12 are treated
     *                     as 1e-12 so the division stays defined.
     * @return Position (order 0) or d^k B / dt^k.
     */
    Point EvaluateWithDuration(int order, double time, double duration) const {
        const double safe_duration = std::max(duration, 1e-12);
        const double parameter = ClampUnit(time / safe_duration);
        Point value = EvaluateDerivative(order, parameter);
        if (order <= 0) {
            return value;
        }
        double divisor = 1.0;
        for (int k = 0; k < order; ++k) {
            divisor *= safe_duration;
        }
        return value * (1.0 / divisor);
    }

    /**
     * @brief Uniform samples from u = 0 to u = 1, inclusive.
     *
     * @param[in] sample_count Number of samples; values below 2 become 2.
     */
    std::vector<Point> Sample(int sample_count) const {
        const int count = std::max(2, sample_count);
        std::vector<Point> samples(static_cast<std::size_t>(count));
        const double denom = static_cast<double>(count - 1);
        for (int i = 0; i < count; ++i) {
            samples[static_cast<std::size_t>(i)] =
                Evaluate(static_cast<double>(i) / denom);
        }
        return samples;
    }

private:
    static double ClampUnit(double value) {
        return std::min(1.0, std::max(0.0, value));
    }

    /**
     * @brief de Casteljau on the first @p degree + 1 entries of @p points.
     */
    static Point DeCasteljau(const std::array<Point, kControlPointCount>& points,
                             int degree, double parameter) {
        if (degree <= 0) {
            return points[0];
        }
        std::array<Point, kControlPointCount> work = points;
        for (int level = 1; level <= degree; ++level) {
            const int last = degree - level;
            for (int i = 0; i <= last; ++i) {
                work[static_cast<std::size_t>(i)] =
                    MixPoints(work[static_cast<std::size_t>(i)],
                              work[static_cast<std::size_t>(i + 1)], parameter);
            }
        }
        return work[0];
    }

    std::array<Point, kControlPointCount> control_points_;
};

/**
 * @brief Concatenated cubic Bézier pieces with a shared global parameter in [0, 1].
 *
 * Segment i occupies the parameter interval [i/n, (i+1)/n]. Adjacent pieces are
 * not forced to be C1; the caller must match P3 of piece i with P0 of piece i+1
 * for C0, and the usual interior-handle condition for C1.
 */
template <typename Point>
class PiecewiseCubicBezier {
public:
    PiecewiseCubicBezier() = default;

    /**
     * @brief Construct from an ordered list of cubic pieces.
     *
     * @param[in] segments Pieces from start to end. Empty is allowed; Evaluate
     *                     then returns a zero point.
     */
    explicit PiecewiseCubicBezier(std::vector<CubicBezier<Point>> segments)
        : segments_(std::move(segments)) {}

    /**
     * @brief Number of cubic pieces.
     */
    int segment_count() const { return static_cast<int>(segments_.size()); }

    /**
     * @brief Pieces in order (const).
     */
    const std::vector<CubicBezier<Point>>& segments() const { return segments_; }

    /**
     * @brief Pieces in order (mutable).
     */
    std::vector<CubicBezier<Point>>& segments() { return segments_; }

    /**
     * @brief Position at a global parameter in [0, 1].
     *
     * @param[in] parameter 0 is the start of the first piece, 1 the end of the
     *                      last. Clamped to [0, 1].
     */
    Point Evaluate(double parameter) const {
        return EvaluateDerivative(0, parameter);
    }

    /**
     * @brief u-derivative of order @p order on the active piece.
     *
     * The derivative is with respect to the local piece parameter, not the
     * global parameter. Multiply by n (segment count) to convert d/d(global).
     *
     * @param[in] order 0 through 3.
     * @param[in] parameter Global parameter in [0, 1], clamped.
     */
    Point EvaluateDerivative(int order, double parameter) const {
        if (segments_.empty()) {
            return ZeroPoint<Point>();
        }
        const double u = std::min(1.0, std::max(0.0, parameter));
        const int count = static_cast<int>(segments_.size());
        double scaled = u * static_cast<double>(count);
        int index = static_cast<int>(std::floor(scaled));
        if (index >= count) {
            index = count - 1;
            scaled = static_cast<double>(count);
        }
        const double local = scaled - static_cast<double>(index);
        return segments_[static_cast<std::size_t>(index)].EvaluateDerivative(
            order, local);
    }

private:
    std::vector<CubicBezier<Point>> segments_;
};

}  // namespace math
}  // namespace common
}  // namespace autonomy
