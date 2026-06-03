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

#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Creates a Ceres cost penalizing longitudinal and angular acceleration
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param weight_acc_x Weight for linear acceleration residual
 * @param weight_acc_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationCostFunction(const TimedElasticBandConfig& config,
                                                    double weight_acc_x,
                                                    double weight_acc_theta);

/**
 * @brief Creates a Ceres cost for the start segment using initial velocities
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param initial_velocity_x Initial longitudinal velocity in m/s
 * @param initial_angular_velocity Initial angular velocity in rad/s
 * @param weight_acc_x Weight for linear acceleration residual
 * @param weight_acc_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationStartCostFunction(
    const TimedElasticBandConfig& config, double initial_velocity_x,
    double initial_angular_velocity, double weight_acc_x,
    double weight_acc_theta);

/**
 * @brief Creates a Ceres cost for the goal segment using goal velocities
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param goal_velocity_x Goal longitudinal velocity in m/s
 * @param goal_angular_velocity Goal angular velocity in rad/s
 * @param weight_acc_x Weight for linear acceleration residual
 * @param weight_acc_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationGoalCostFunction(
    const TimedElasticBandConfig& config, double goal_velocity_x,
    double goal_angular_velocity, double weight_acc_x, double weight_acc_theta);

/**
 * @brief Creates a holonomic Ceres acceleration cost along the band
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param weight_x Weight for x acceleration residual
 * @param weight_y Weight for y acceleration residual
 * @param weight_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationHolonomicCostFunction(
    const TimedElasticBandConfig& config, double weight_x, double weight_y,
    double weight_theta);

/**
 * @brief Creates a holonomic start-segment acceleration cost
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param initial_velocity_x Initial longitudinal velocity in m/s
 * @param initial_velocity_y Initial lateral velocity in m/s
 * @param initial_angular_velocity Initial angular velocity in rad/s
 * @param weight_x Weight for x acceleration residual
 * @param weight_y Weight for y acceleration residual
 * @param weight_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationHolonomicStartCostFunction(
    const TimedElasticBandConfig& config, double initial_velocity_x,
    double initial_velocity_y, double initial_angular_velocity, double weight_x,
    double weight_y, double weight_theta);

/**
 * @brief Creates a holonomic goal-segment acceleration cost
 *
 * @param config TEB configuration with robot limits and penalty epsilon
 * @param goal_velocity_x Goal longitudinal velocity in m/s
 * @param goal_velocity_y Goal lateral velocity in m/s
 * @param goal_angular_velocity Goal angular velocity in rad/s
 * @param weight_x Weight for x acceleration residual
 * @param weight_y Weight for y acceleration residual
 * @param weight_theta Weight for angular acceleration residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateAccelerationHolonomicGoalCostFunction(
    const TimedElasticBandConfig& config, double goal_velocity_x, double goal_velocity_y,
    double goal_angular_velocity, double weight_x, double weight_y,
    double weight_theta);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
