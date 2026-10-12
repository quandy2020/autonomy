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

#pragma once

/**
 * @file skew_symmetric_matrix.hpp
 * @brief Get the skew symmetric representation of a vector.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * Get the skew symmetric representation of a vector
 *
 * v_hat = [0.0, -v.z(), v.y(),
 *          v.z(), 0.0, -v.x(),
 *          -v.y(), v.x(), 0.0];
 *
 * @param [in] vector: 3x1
 * @return matrix 3x3
 */
template <typename SCALAR>
/**
 * @brief Skew Symmetric Matrix.
 * @param [in] v: V.
 * @return Result value.
 */
Eigen::Matrix<SCALAR, 3, 3> SkewSymmetricMatrix(Eigen::Matrix<SCALAR, 3, 1> v) {
  Eigen::Matrix<SCALAR, 3, 3> skewSymmetricMatrix;
  // clang-format off
  /**
   * @brief SCALAR.
   * @return Result value.
   */
  SkewSymmetricMatrix << SCALAR(0.0),       -v(2),        v(1),
                                v(2), SCALAR(0.0),       -v(0),
                               -v(1),        v(0), SCALAR(0.0);
  // clan-format on
  return skewSymmetricMatrix;
}

}  // namespace bolza
