// Copyright 2025 Aarav Gupta
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

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_OMNI_WHEEL_DRIVE__ODOMETRY_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_OMNI_WHEEL_DRIVE__ODOMETRY_HPP_

#include <Eigen/Dense>
#include <vector>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace omni_wheel_drive
{

class Odometry
{
public:
  Odometry();

  bool updateFromPos(const std::vector<double> & wheels_pos, double time_sec);
  bool updateFromVel(const std::vector<double> & wheels_vel, double time_sec);
  bool updateOpenLoop(
    double linear_x_vel, double linear_y_vel, double angular_vel, double time_sec);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearXVel() const { return linear_x_vel_; }
  double getLinearYVel() const { return linear_y_vel_; }
  double getAngularVel() const { return angular_vel_; }

  void setParams(
    double robot_radius, double wheel_radius, double wheel_offset, size_t wheel_count);

private:
  Eigen::Vector3d compute_robot_velocity(const std::vector<double> & wheels_vel) const;
  void integrate(double dx, double dy, double dheading);

  double timestamp_;

  double x_;
  double y_;
  double heading_;

  double linear_x_vel_;
  double linear_y_vel_;
  double angular_vel_;

  double robot_radius_;
  double wheel_radius_;
  double wheel_offset_;

  std::vector<double> wheels_old_pos_;
};

}  // namespace omni_wheel_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_OMNI_WHEEL_DRIVE__ODOMETRY_HPP_
