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
 * @file performance_indices_rollout.hpp
 * @brief Computes the accumulated cost over the given trajectory.
 */


#include <functional>

#include <bolza/core/types.hpp>

namespace bolza {
namespace PerformanceIndicesRollout {

using cost_wraper_t = std::function<scalar_t(scalar_t, const vector_t&, const vector_t&)>;
using constraints_wraper_t = std::function<vector_t(scalar_t, const vector_t&, const vector_t&)>;

/**
 * @brief Computes the accumulated cost over the given trajectory.
 * @param [in] costWraper: Cost Wraper.
 * @param [in] timeTrajectory: Time Trajectory.
 * @param [in] stateTrajectory: State Trajectory.
 * @param [in] inputTrajectory: Input Trajectory.
 * @return Result value.
 */
scalar_t RolloutCost(cost_wraper_t costWraper, const scalar_array_t& timeTrajectory, const vector_array_t& stateTrajectory,
                     const vector_array_t& inputTrajectory);

/**
 * @brief Computes the ISE (Integral of Square Error) of constraint over the given trajectory.
 * @param [in] constraintWraper: Constraint Wraper.
 * @param [in] timeTrajectory: Time Trajectory.
 * @param [in] stateTrajectory: State Trajectory.
 * @param [in] inputTrajectory: Input Trajectory.
 * @return Result value.
 */
scalar_t RolloutConstraint(constraints_wraper_t constraintWraper, const scalar_array_t& timeTrajectory,
                           const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory);

}  // namespace PerformanceIndicesRollout
}  // namespace bolza
