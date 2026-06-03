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
 * @brief Creates a Ceres cost enforcing per-segment velocity limits on a
 * diff-drive robot
 *
 * @param config TEB configuration with velocity and penalty limits
 * @param weight_vx Weight for longitudinal velocity bound
 * @param weight_omega Weight for angular velocity bound
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateVelocityCostFunction(const TimedElasticBandConfig& config,
                                                double weight_vx,
                                                double weight_omega);

/**
 * @brief Creates a Ceres cost enforcing holonomic per-segment velocity limits
 *
 * @param config TEB configuration with velocity and penalty limits
 * @param weight_vx Weight for longitudinal velocity bound
 * @param weight_vy Weight for lateral velocity bound
 * @param weight_omega Weight for angular velocity bound
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateVelocityHolonomicCostFunction(
    const TimedElasticBandConfig& config, double weight_vx, double weight_vy,
    double weight_omega);

/**
 * @brief Creates a Ceres cost scaling velocity limits by obstacle proximity
 *
 * Uses numeric differentiation for distance queries.
 *
 * @param config TEB configuration with proximity ratio parameters
 * @param robot_model Robot footprint model for distance computation
 * @param obstacle Non-owning obstacle used for proximity scaling
 * @param weight Residual weight
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateVelocityObstacleRatioCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
