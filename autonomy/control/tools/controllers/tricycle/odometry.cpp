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

#include "autonomy/control/tools/controllers/tricycle/odometry.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace tricycle
{

Odometry::Odometry(size_t velocity_rolling_window_size)
: x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(double Ws, double alpha, double dt_sec)
{
  // http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  const double Vs = Ws * wheel_radius_;
  const double Vx = Vs * std::cos(alpha);
  const double theta_dot = Vs * std::sin(alpha) / wheelbase_;

  integrateExact(Vx * dt_sec, theta_dot * dt_sec);

  linear_accumulator_.accumulate(Vx);
  angular_accumulator_.accumulate(theta_dot);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, double dt_sec)
{
  linear_ = linear;
  angular_ = angular;
  integrateExact(linear * dt_sec, angular * dt_sec);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  resetAccumulators();
}

void Odometry::setWheelParams(double wheelbase, double wheel_radius)
{
  wheelbase_ = wheelbase;
  wheel_radius_ = wheel_radius;
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

}  // namespace tricycle
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy
