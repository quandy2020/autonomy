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

#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Creates a minimum-distance obstacle cost at a single pose
 *
 * Uses numeric differentiation for footprint distance queries.
 *
 * @param config TEB configuration with obstacle and penalty parameters
 * @param robot_model Robot footprint model for distance computation
 * @param obstacle Non-owning obstacle to avoid
 * @param weight Residual weight
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight);

/**
 * @brief Creates an obstacle cost with an inflation-distance residual
 *
 * @param config TEB configuration with obstacle and penalty parameters
 * @param robot_model Robot footprint model for distance computation
 * @param obstacle Non-owning obstacle to avoid
 * @param weight_obstacle Weight for minimum-distance residual
 * @param weight_inflation Weight for inflation band residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateInflatedObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight_obstacle, double weight_inflation);

/**
 * @brief Creates a spatio-temporal clearance cost for a dynamic obstacle
 *
 * @param config TEB configuration with dynamic obstacle parameters
 * @param robot_model Robot footprint model for distance computation
 * @param obstacle Non-owning dynamic obstacle to avoid
 * @param time_offset Time along the band when distance is evaluated in seconds
 * @param weight_obstacle Weight for minimum-distance residual
 * @param weight_inflation Weight for dynamic inflation residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateDynamicObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double time_offset, double weight_obstacle,
    double weight_inflation);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
