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

#pragma once

#include <ceres/ceres.h>

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Creates a time-optimal Ceres cost penalizing segment durations
 *
 * @param weight Multiplier applied to each time-difference residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateTimeOptimalCostFunction(double weight);

/**
 * @brief Creates a shortest-path Ceres cost between consecutive poses
 *
 * @param weight Multiplier applied to Euclidean segment length residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateShortestPathCostFunction(double weight);

/**
 * @brief Creates a Ceres cost pulling a pose toward a via point
 *
 * @param via_point_x Target x coordinate in the world frame
 * @param via_point_y Target y coordinate in the world frame
 * @param weight Residual weight
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateViaPointCostFunction(double via_point_x,
                                                double via_point_y,
                                                double weight);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
