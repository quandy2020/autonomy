// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Ported into autonomy::control::tools::controllers
// Thin twist → steering/traction command wrapper over SteeringKinematics::get_commands.

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__TWIST_TO_STEERING_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__TWIST_TO_STEERING_HPP_

#include <tuple>
#include <vector>

#include "autonomy/control/tools/controllers/steering/steering_kinematics.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace steering
{

/**
 * \brief Map planar twist `(v, ω)` to traction / steering joint commands.
 *
 * Configured via `SteeringKinematics::set_odometry_type`:
 * - `BICYCLE_CONFIG`  — single traction + single steer
 * - `TRICYCLE_CONFIG` — dual traction + single steer
 * - `ACKERMANN_CONFIG` — dual traction + dual steer
 *
 * Returns `{traction_commands, steering_commands}` from
 * `SteeringKinematics::get_commands`.
 */
inline std::tuple<std::vector<double>, std::vector<double>> twist_to_steering(
  SteeringKinematics & kinematics, double v, double omega, bool open_loop = true,
  bool reduce_wheel_speed_until_steering_reached = false)
{
  return kinematics.get_commands(v, omega, open_loop, reduce_wheel_speed_until_steering_reached);
}

}  // namespace steering
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__TWIST_TO_STEERING_HPP_
