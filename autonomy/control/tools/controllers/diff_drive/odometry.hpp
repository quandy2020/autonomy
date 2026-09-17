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
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

// Ported into autonomy::control::tools::controllers

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__ODOMETRY_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__ODOMETRY_HPP_

#include <cstddef>

#include "autonomy/control/tools/rolling_mean.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace diff_drive
{

class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(double time_sec);
  bool update(double left_pos, double right_pos, double time_sec);
  bool updateFromVelocity(double left_vel, double right_vel, double time_sec);
  void updateOpenLoop(double linear, double angular, double time_sec);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = autonomy::control::tools::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  double timestamp_;

  double x_;
  double y_;
  double heading_;

  double linear_;
  double angular_;

  double wheel_separation_;
  double left_wheel_radius_;
  double right_wheel_radius_;

  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace diff_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__ODOMETRY_HPP_
