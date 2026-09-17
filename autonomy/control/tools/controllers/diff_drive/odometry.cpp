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

#include "autonomy/control/tools/controllers/diff_drive/odometry.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace diff_drive
{

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  left_wheel_old_pos_(0.0),
  right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(double time_sec)
{
  resetAccumulators();
  timestamp_ = time_sec;
}

bool Odometry::update(double left_pos, double right_pos, double time_sec)
{
  const double dt = time_sec - timestamp_;
  if (dt < 0.0001)
  {
    return false;
  }

  const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
  const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

  const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_old_pos_;
  const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time_sec);

  return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, double time_sec)
{
  const double dt = time_sec - timestamp_;
  if (dt < 0.0001)
  {
    return false;
  }

  const double linear = (left_vel + right_vel) * 0.5;
  const double angular = (right_vel - left_vel) / wheel_separation_;

  integrateExact(linear, angular);

  timestamp_ = time_sec;

  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, double time_sec)
{
  linear_ = linear;
  angular_ = angular;

  const double dt = time_sec - timestamp_;
  timestamp_ = time_sec;
  integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double left_wheel_radius, double right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  x_ += linear * std::cos(direction);
  y_ += linear * std::sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (std::fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (std::sin(heading_) - std::sin(heading_old));
    y_ += -r * (std::cos(heading_) - std::cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy
