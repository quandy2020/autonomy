/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <utility>

namespace bolza {
namespace bilinear_interpolation {

/******************************************************************************************************/

/**
 * @file bilinear_interpolation.hpp
 * @brief bilinear interpolation (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
/**
 * @brief Get Value.
 * @param [in] resolution: Resolution.
 * @param [in] referenceCorner: Reference Corner.
 * @param [in] cornerValues: Corner Values.
 * @param [in] position: Position.
 * @return Result value.
 */
Scalar GetValue(Scalar resolution, const Eigen::Matrix<Scalar, 2, 1>& referenceCorner, const std::array<Scalar, 4>& cornerValues,
                const Eigen::Matrix<Scalar, 2, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 4> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = 1 - v[0];
  v[3] = 1 - v[1];
  // (1 - x)(1 - y) f_00 + x (1 - y) f_10 + (1 - x) y f_01 + x y f_11
  return cornerValues[0] * v[2] * v[3] + cornerValues[1] * v[0] * v[3] + cornerValues[2] * v[2] * v[1] + cornerValues[3] * v[0] * v[1];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
/**
 * @brief Get Linear Approximation.
 * @param [in] resolution: Resolution.
 * @param [in] referenceCorner: Reference Corner.
 * @param [in] cornerValues: Corner Values.
 * @param [in] position: Position.
 * @return Result value.
 */
std::pair<Scalar, Eigen::Matrix<Scalar, 2, 1>> GetLinearApproximation(Scalar resolution, const Eigen::Matrix<Scalar, 2, 1>& referenceCorner,
                                                                      const std::array<Scalar, 4>& cornerValues,
                                                                      const Eigen::Matrix<Scalar, 2, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 6> v;
  v[0] = (position.y() - referenceCorner.y()) * r_inv;
  v[1] = (position.x() - referenceCorner.x()) * r_inv;
  v[2] = 1.0 - v[0];
  v[3] = 1.0 - v[1];
  v[4] = cornerValues[2] * v[3];  // (1 - x) f_01
  v[5] = cornerValues[3] * v[1];  // x f_11
  v[3] = cornerValues[0] * v[3];  // (1 - x) f_00
  v[1] = cornerValues[1] * v[1];  // x f_10

  // x ( 1 - y) f_10 + (1 - x) (1 - y) f_00 + (1 - x) y f_01 + x y f_11
  const Scalar value = v[1] * v[2] + v[3] * v[2] + v[4] * v[0] + v[5] * v[0];

  Eigen::Matrix<Scalar, 2, 1> gradient;
  // (1 - y) (f_10 - f_00) + y (f_11 - f_01)
  gradient.x() = (v[2] * (cornerValues[1] - cornerValues[0]) + v[0] * (cornerValues[3] - cornerValues[2])) * r_inv;
  // (1 - x) (f_01 - f_00) + x (f_11 - f_10)
  gradient.y() = (v[4] + v[5] - (v[3] + v[1])) * r_inv;

  return {value, gradient};
}

}  // namespace bilinear_interpolation
}  // namespace bolza
