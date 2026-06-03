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
 * @brief Creates a Ceres cost favoring a preferred rotation direction
 *
 * Penalizes yaw changes opposite to the preferred direction on early trajectory
 * segments.
 *
 * @param rotation_sign +1 for left turn, -1 for right turn (see PreferredRotationDirection)
 * @param weight Residual weight
 * @return ceres::CostFunction* Newly allocated cost function; caller owns it
 */
ceres::CostFunction* CreatePreferredRotationDirectionCostFunction(
    double rotation_sign, double weight);

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
