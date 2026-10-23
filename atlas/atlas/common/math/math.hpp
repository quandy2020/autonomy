/*
 * Copyright 2026 The Openbot Authors
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

#ifndef ATLAS_COMMON_MATH_HPP_
#define ATLAS_COMMON_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace atlas {
namespace common {

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vector);

// Rodrigues. Near-zero angles return identity.
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& angle_axis);

// Inverse of ExpSO3. Input is treated as a rotation matrix.
Eigen::Vector3d LogSO3(const Eigen::Matrix3d& rotation);

// se(3) coordinates are [ω, ρ]: rotation then translation.
Eigen::Isometry3d ExpSE3(const Eigen::Matrix<double, 6, 1>& twist);

Eigen::Matrix<double, 6, 1> LogSE3(const Eigen::Isometry3d& pose);

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_MATH_HPP_
