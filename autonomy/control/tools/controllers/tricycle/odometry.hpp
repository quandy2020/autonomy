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
// rclcpp::Duration → double dt_sec; RollingMean → tools::RollingMeanAccumulator

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__ODOMETRY_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__ODOMETRY_HPP_

#include <cstddef>

#include "autonomy/control/tools/rolling_mean.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace tricycle
{

class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Update from traction wheel angular speed and steer angle.
   * \param Ws     Traction wheel angular velocity [rad/s]
   * \param alpha  Steering angle [rad]
   * \param dt_sec Timestep [s]
   */
  bool update(double Ws, double alpha, double dt_sec);
  void updateOpenLoop(double linear, double angular, double dt_sec);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheelbase, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = autonomy::control::tools::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  double x_;
  double y_;
  double heading_;

  double linear_;
  double angular_;

  double wheelbase_;
  double wheel_radius_;

  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace tricycle
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_TRICYCLE__ODOMETRY_HPP_
