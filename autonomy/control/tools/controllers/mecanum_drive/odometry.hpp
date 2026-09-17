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

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_MECANUM_DRIVE__ODOMETRY_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_MECANUM_DRIVE__ODOMETRY_HPP_

#include <array>
#include <functional>

#define AUTONOMY_MECANUM_PLANAR_POINT_DIM 3

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace mecanum_drive
{

class Odometry
{
public:
  typedef std::function<void(double, double, double)> IntegrationFunction;

  Odometry();

  void init(double time_sec, std::array<double, AUTONOMY_MECANUM_PLANAR_POINT_DIM> base_frame_offset);

  bool update(
    double wheel_front_left_vel, double wheel_rear_left_vel, double wheel_rear_right_vel,
    double wheel_front_right_vel, double dt);

  double getX() const { return position_x_in_base_frame_; }
  double getY() const { return position_y_in_base_frame_; }
  double getRz() const { return orientation_z_in_base_frame_; }
  double getVx() const { return velocity_in_base_frame_linear_x_; }
  double getVy() const { return velocity_in_base_frame_linear_y_; }
  double getWz() const { return velocity_in_base_frame_angular_z_; }

  const std::array<double, AUTONOMY_MECANUM_PLANAR_POINT_DIM> & getBaseFrameOffset() const
  {
    return base_frame_offset_;
  }

  void setWheelsParams(
    double sum_of_robot_center_projection_on_X_Y_axis, double wheels_radius);

private:
  double timestamp_;

  std::array<double, AUTONOMY_MECANUM_PLANAR_POINT_DIM> base_frame_offset_;

  double position_x_in_base_frame_;
  double position_y_in_base_frame_;
  double orientation_z_in_base_frame_;

  double velocity_in_base_frame_linear_x_;
  double velocity_in_base_frame_linear_y_;
  double velocity_in_base_frame_angular_z_;

  double sum_of_robot_center_projection_on_X_Y_axis_;
  double wheels_radius_;
};

}  // namespace mecanum_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_MECANUM_DRIVE__ODOMETRY_HPP_
