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
 * @file trilinear_interpolation.hpp
 * @brief Compute the value of a function at a queried position using tri-linear interpolation on a 3D-grid.
 */


#include <array>

#include <bolza/core/types.hpp>

namespace bolza {
namespace trilinear_interpolation {

/**
 * Compute the value of a function at a queried position using tri-linear interpolation on a 3D-grid.
 *
 * @param [in] resolution: The resolution of the grid.
 * @param [in] referenceCorner: The reference position on the 3-D grid closest to the point.
 * @param [in] cornerValues: The values around the reference corner, in the order:
 *  (0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1).
 * @param [in] position: The queried position.
 * @tparam Scalar : The Scalar type.
 * @return Scalar : The interpolated function's value at the queried position.
 */
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
                const Eigen::Matrix<Scalar, 3, 1>& position);

/**
 * Computes a first-order approximation of the function at a queried position using tri-linear interpolation on a 3D-grid.
 *
 * @param [in] resolution: The resolution of the grid.
 * @param [in] referenceCorner: The reference position on the 3-D grid closest to the point.
 * @param [in] cornerValues: The values around the reference corner, in the order:
 *  (0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1).
 * @param [in] position: The queried position.
 * @tparam Scalar : The Scalar type.
 * @return std::pair<Scalar, Eigen::Matrix<Scalar, 3, 1>> : A tuple containing the value and jacobian.
 */
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
                                                                      const Eigen::Matrix<Scalar, 3, 1>& position);

}  // namespace trilinear_interpolation
}  // namespace bolza

#include "implementation/trilinear_interpolation.hpp"
