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

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__TRACTION_LIMITER_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__TRACTION_LIMITER_HPP_

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace tricycle
{

class TractionLimiter
{
public:
  /**
   * \brief Constructor
   *
   * Parameters are applied symmetrically for both directions.
   *
   * \warning
   * - Setting min_velocity: the robot can't stand still
   * - Setting min_deceleration/min_acceleration: the robot can't move with constant velocity
   * - Setting min_jerk: the robot can't move with constant acceleration
   */
  TractionLimiter(
    double min_velocity = NAN, double max_velocity = NAN, double min_acceleration = NAN,
    double max_acceleration = NAN, double min_deceleration = NAN, double max_deceleration = NAN,
    double min_jerk = NAN, double max_jerk = NAN);

  /**
   * \brief Limit the velocity and acceleration
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  double limit_velocity(double & v);

  double limit_acceleration(double & v, double v0, double dt);

  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  double min_velocity_;
  double max_velocity_;

  double min_acceleration_;
  double max_acceleration_;

  double min_deceleration_;
  double max_deceleration_;

  double min_jerk_;
  double max_jerk_;
};

}  // namespace tricycle
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__TRACTION_LIMITER_HPP_
