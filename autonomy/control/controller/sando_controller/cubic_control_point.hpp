/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 * @file cubic_control_point.hpp
 * @brief Control-point maps and sparse linear rows for one cubic piece.
 *
 * A cubic on u in [0, 1] is p = c0 + c1 u + c2 u^2 + c3 u^3. Position, velocity,
 * and acceleration rows are the Bézier control points of that monomial and of
 * its derivatives. Jerk is the constant 6 c3 / T^3. Constraining those control
 * points inside a convex set constrains the polynomial, because their convex
 * hull contains the curve.
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Number of monomial coefficients of one cubic, and the position control-point count.
 */
constexpr int kCubicCoefficientCount = 4;

/**
 * @brief Control points of one cubic piece, and the sparse rows that constrain them.
 */
class CubicControlPoint {
 public:
  /**
   * @brief One sparse linear form a·x <= rhs, or a·x = rhs when used as an equality.
   *
   * terms stores (column, coefficient) pairs. Coefficients smaller than 1e-12
   * are dropped so a structural zero does not enter the MIQP matrix.
   */
  struct Row {
    std::vector<std::pair<int, double>> terms;  ///< Nonzero (column, coefficient) pairs.
    double rhs{0.0};                            ///< Right-hand side. Equalities use the same field.
  };

  /**
   * @brief Column of one monomial coefficient in the stacked decision vector.
   *
   * The layout is segment-major, then axis (x then y), then power 0..3:
   * index = (segment * 2 + axis) * 4 + power. Binary variables of the MIQP
   * start after segments * 8 coefficients.
   *
   * @param segment Piece index, zero-based.
   * @param axis 0 for x, 1 for y.
   * @param power Monomial power, 0 through 3.
   * @return Column index into the Hessian and the constraint rows.
   */
  static int ComputeCoefficientIndex(int segment, int axis, int power);

  /**
   * @brief Evaluate one axis of a cubic, or its derivative, on u in [0, 1].
   *
   * This is the normalized derivative. The physical derivative of order k is
   * this value divided by duration^k. deriv = 0 is position, 1 velocity, 2
   * acceleration, 3 jerk.
   *
   * @param coefficients Four monomial coefficients, power 0 through 3, of one axis.
   * @param deriv Derivative order, 0 through 3.
   * @param normalized_time Normalized time in [0, 1].
   * @return The normalized derivative.
   */
  static double EvaluateMonomial(const double* coefficients, int deriv, double normalized_time);

  /**
   * @brief Append one nonzero coefficient to a sparse row.
   * @param row Row under construction. Required.
   * @param index Decision-variable column.
   * @param value Coefficient. Dropped when its absolute value is at most 1e-12.
   */
  static void AddTerm(Row* row, int index, double value);

  /**
   * @brief Add scale * src into dst. The right-hand side of src is not copied.
   * @param dst Destination row. Required.
   * @param src Source row.
   * @param scale Multiplier, including a negative sign that flips a bound.
   */
  static void AddScaledRow(Row* dst, const Row& src, double scale);

  /**
   * @brief Test a planar vector against an L1, L2, or L-infinity bound with five percent slack.
   * @param x First component.
   * @param y Second component.
   * @param limit Bound. The accepted magnitude is limit * 1.05 + 1e-6.
   * @param norm "L1", "L2", or anything else for L-infinity.
   * @return True when the vector lies inside the slackened bound.
   */
  static bool SatisfiesNormBound(double x, double y, double limit, const std::string& norm);

  /**
   * @brief Append linear inequalities that outer-approximate a norm bound on (x, y).
   *
   * L-infinity writes the four bounds ±x <= limit and ±y <= limit. L1 writes the
   * four diagonal bounds ±x ± y <= limit. L2 writes eight tangent inequalities
   * of the unit circle. Each row is sign_x * x_row + sign_y * y_row <= limit.
   *
   * @param ineq Inequality list appended in place. Required.
   * @param x Sparse expression for the x component.
   * @param y Sparse expression for the y component.
   * @param limit Bound, same units as the expression.
   * @param norm "L1", "L2", or anything else for L-infinity.
   */
  static void AppendNormBounds(std::vector<Row>* ineq, const Row& x, const Row& y, double limit, const std::string& norm);

  /**
   * @brief Sparse row of one cubic Bézier position control point.
   * @param segment Piece index.
   * @param axis 0 for x, 1 for y.
   * @param point Control-point index, 0 through 3.
   * @return Linear form in the monomial coefficients. The right-hand side is 0.
   */
  static Row MakePositionControlRow(int segment, int axis, int point);

  /**
   * @brief Sparse row of one velocity control point, in m/s.
   * @param segment Piece index.
   * @param axis 0 for x, 1 for y.
   * @param point Control-point index, 0 through 2.
   * @param duration Piece duration T, seconds. Divides the derivative.
   * @return Linear form whose value is the control point.
   */
  static Row MakeVelocityControlRow(int segment, int axis, int point, double duration);

  /**
   * @brief Sparse row of one acceleration control point, in m/s^2.
   * @param segment Piece index.
   * @param axis 0 for x, 1 for y.
   * @param point Control-point index, 0 or 1.
   * @param duration Piece duration T, seconds. The row divides by T^2.
   * @return Linear form whose value is the control point.
   */
  static Row MakeAccelerationControlRow(int segment, int axis, int point, double duration);
};

using Row = CubicControlPoint::Row;

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
