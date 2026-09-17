// Copyright 2020 PAL Robotics S.L.
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
 * Author: Enrique Fernández
 */

// Ported into autonomy::control::tools::controllers

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__SPEED_LIMITER_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__SPEED_LIMITER_HPP_

#include <limits>

#include "autonomy/control/tools/rate_limiter.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace diff_drive
{

/**
 * \brief Thin wrapper around tools::RateLimiter for diff-drive velocity limits.
 */
class SpeedLimiter
{
public:
  explicit SpeedLimiter(
    bool has_velocity_limits = true, bool has_acceleration_limits = true,
    bool has_jerk_limits = true, double min_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_deceleration = std::numeric_limits<double>::quiet_NaN(),
    double max_acceleration = std::numeric_limits<double>::quiet_NaN(),
    double min_jerk = std::numeric_limits<double>::quiet_NaN(),
    double max_jerk = std::numeric_limits<double>::quiet_NaN())
  {
    if (!has_velocity_limits)
    {
      min_velocity = max_velocity = std::numeric_limits<double>::quiet_NaN();
    }
    if (!has_acceleration_limits)
    {
      max_deceleration = max_acceleration = std::numeric_limits<double>::quiet_NaN();
    }
    if (!has_jerk_limits)
    {
      min_jerk = max_jerk = std::numeric_limits<double>::quiet_NaN();
    }
    speed_limiter_ = autonomy::control::tools::RateLimiter<double>(
      min_velocity, max_velocity, max_deceleration, max_acceleration,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), min_jerk,
      max_jerk);
  }

  explicit SpeedLimiter(
    double min_velocity, double max_velocity, double max_acceleration_reverse,
    double max_acceleration, double max_deceleration, double max_deceleration_reverse,
    double min_jerk, double max_jerk)
  {
    speed_limiter_ = autonomy::control::tools::RateLimiter<double>(
      min_velocity, max_velocity, max_acceleration_reverse, max_acceleration, max_deceleration,
      max_deceleration_reverse, min_jerk, max_jerk);
  }

  double limit(double & v, double v0, double v1, double dt)
  {
    return speed_limiter_.limit(v, v0, v1, dt);
  }

  double limit_velocity(double & v) { return speed_limiter_.limit_value(v); }

  double limit_acceleration(double & v, double v0, double dt)
  {
    return speed_limiter_.limit_first_derivative(v, v0, dt);
  }

  double limit_jerk(double & v, double v0, double v1, double dt)
  {
    return speed_limiter_.limit_second_derivative(v, v0, v1, dt);
  }

private:
  autonomy::control::tools::RateLimiter<double> speed_limiter_;
};

}  // namespace diff_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__SPEED_LIMITER_HPP_
