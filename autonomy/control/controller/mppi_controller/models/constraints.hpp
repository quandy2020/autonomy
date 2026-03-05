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

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace models {

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints {
  float vx_max;
  float vx_min;
  float vy;
  float wz;
  float ax_max;
  float ax_min;
  float ay_min;
  float ay_max;
  float az_max;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd {
  float vx;
  float vy;
  float wz;
};

}  // namespace models
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy