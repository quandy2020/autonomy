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
 * @brief Creates a diff-drive kinematics cost for non-holonomic motion
 *
 * @param weight_nh Weight for non-holonomic constraint residual
 * @param weight_forward Weight for forward-motion constraint residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateKinematicsDiffDriveCostFunction(
    double weight_nh, double weight_forward);

/**
 * @brief Creates a car-like kinematics cost with minimum turning radius
 *
 * @param config TEB configuration with turning radius and trajectory options
 * @param weight_nh Weight for non-holonomic constraint residual
 * @param weight_radius Weight for minimum turning-radius residual
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreateKinematicsCarlikeCostFunction(
    const TimedElasticBandConfig& config, double weight_nh, double weight_radius);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
