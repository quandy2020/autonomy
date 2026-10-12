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
namespace trilinear_interpolation {

/******************************************************************************************************/

/**
 * @file trilinear_interpolation.hpp
 * @brief trilinear interpolation (bolza).
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
Scalar GetValue(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner, const std::array<Scalar, 8>& cornerValues,
                const Eigen::Matrix<Scalar, 3, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 10> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = (position.z() - referenceCorner.z()) * r_inv;
  v[3] = 1 - v[0];
  v[4] = 1 - v[1];
  v[5] = 1 - v[2];
  v[6] = v[3] * cornerValues[0] + v[0] * cornerValues[1];  // f_00 = (1 - x) f_000 + x f_100
  v[7] = v[3] * cornerValues[2] + v[0] * cornerValues[3];  // f_10 = (1 - x) f_010 + x f_110
  v[8] = v[3] * cornerValues[4] + v[0] * cornerValues[5];  // f_01 = (1 - x) f_001 + x f_101
  v[9] = v[3] * cornerValues[6] + v[0] * cornerValues[7];  // f_11 = (1 - x) f_011 + x f_111
  // (1 - z) ((1 - y) f_00 + y f_10) + z ((1 - y) f_01 + y f_11)
  return v[5] * (v[4] * v[6] + v[1] * v[7]) + v[2] * (v[4] * v[8] + v[1] * v[9]);
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
std::pair<Scalar, Eigen::Matrix<Scalar, 3, 1>> GetLinearApproximation(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner,
                                                                      const std::array<Scalar, 8>& cornerValues,
                                                                      const Eigen::Matrix<Scalar, 3, 1>& position) {
  /// auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 12> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = (position.z() - referenceCorner.z()) * r_inv;
  v[3] = 1 - v[0];
  v[4] = 1 - v[1];
  v[5] = 1 - v[2];
  v[6] = v[3] * cornerValues[0] + v[0] * cornerValues[1];  // f_00 = (1 - x) f_000 + x f_100
  v[7] = v[3] * cornerValues[2] + v[0] * cornerValues[3];  // f_10 = (1 - x) f_010 + x f_110
  v[8] = v[3] * cornerValues[4] + v[0] * cornerValues[5];  // f_01 = (1 - x) f_001 + x f_101
  v[9] = v[3] * cornerValues[6] + v[0] * cornerValues[7];  // f_11 = (1 - x) f_011 + x f_111
  v[10] = v[4] * v[6] + v[1] * v[7];                       // f_0 = (1 - y) f_00 + y f_10
  v[11] = v[4] * v[8] + v[1] * v[9];                       // f_1 = (1 - y) f_01 + y f_11

  // f = (1 - z) f_0 + z f_1
  const Scalar value = v[5] * v[10] + v[2] * v[11];

  Eigen::Matrix<Scalar, 3, 1> gradient;
  // df_z = f_11 - f_10
  gradient.z() = (v[11] - v[10]) * r_inv;
  // df_y = (1 - z) (f_10 - f00) - z (f_11 - f_01)
  gradient.y() = (v[5] * (v[7] - v[6]) + v[2] * (v[9] - v[8])) * r_inv;
  // df_x = (1 - z) (1 - y) (f_100 - f_000) + (1 - z) y (f_110 - f_010) + z (1 - y) (f_101 - f_001) + z y (f_111 - f_011)
  gradient.x() = (v[5] * v[4] * (cornerValues[1] - cornerValues[0]) + v[5] * v[1] * (cornerValues[3] - cornerValues[2]) +
                  v[2] * v[4] * (cornerValues[5] - cornerValues[4]) + v[2] * v[1] * (cornerValues[7] - cornerValues[6])) *
                 r_inv;

  return {value, gradient};
}

}  // namespace trilinear_interpolation
}  // namespace bolza
