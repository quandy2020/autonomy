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

#include <Eigen/Core>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"

namespace bolza {
namespace mobile_manipulator {

/**
 * Provides read/write access to the base position.
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> GetBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info);

/**
 * Provides read/write access to the base orientation.
 */
template <typename SCALAR>
Eigen::Quaternion<SCALAR> GetBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info);

/**
 * Provides read/write access to the arm joint angles.
 */
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info);

/**
 * Provides read access to the arm joint angles.
 */
template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info);

}  // namespace mobile_manipulator
}  // namespace bolza

#include "implementation/access_helper_functions_impl.hpp"
