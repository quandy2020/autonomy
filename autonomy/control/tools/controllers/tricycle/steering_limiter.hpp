// Copyright 2022 Pixel Robotics.
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

/*
 * Author: Tony Najjar
 */

// Ported into autonomy::control::tools::controllers::tricycle

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__STEERING_LIMITER_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__STEERING_LIMITER_HPP_

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace tricycle
{

class SteeringLimiter
{
public:
  /**
   * \brief Constructor
   * \param [in] min_position Minimum position [m] or [rad]
   * \param [in] max_position Maximum position [m] or [rad]
   * \param [in] min_velocity Minimum velocity [m/s] or [rad/s]
   * \param [in] max_velocity Maximum velocity [m/s] or [rad/s]
   * \param [in] min_acceleration Minimum acceleration [m/s^2] or [rad/s^2]
   * \param [in] max_acceleration Maximum acceleration [m/s^2] or [rad/s^2]
   */
  SteeringLimiter(
    double min_position = NAN, double max_position = NAN, double min_velocity = NAN,
    double max_velocity = NAN, double min_acceleration = NAN, double max_acceleration = NAN);

  /**
   * \brief Limit the position, velocity and acceleration
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & p, double p0, double p1, double dt);

  double limit_position(double & p);

  double limit_velocity(double & p, double p0, double dt);

  double limit_acceleration(double & p, double p0, double p1, double dt);

private:
  double min_position_;
  double max_position_;

  double min_velocity_;
  double max_velocity_;

  double min_acceleration_;
  double max_acceleration_;
};

}  // namespace tricycle
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__STEERING_LIMITER_HPP_
