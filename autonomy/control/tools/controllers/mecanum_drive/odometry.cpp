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

#include "autonomy/control/tools/controllers/mecanum_drive/odometry.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace mecanum_drive
{

namespace
{
inline void rotate2d(double yaw, double x, double y, double & out_x, double & out_y)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  out_x = c * x - s * y;
  out_y = s * x + c * y;
}
}  // namespace

Odometry::Odometry()
: timestamp_(0.0),
  base_frame_offset_{0.0, 0.0, 0.0},
  position_x_in_base_frame_(0.0),
  position_y_in_base_frame_(0.0),
  orientation_z_in_base_frame_(0.0),
  velocity_in_base_frame_linear_x_(0.0),
  velocity_in_base_frame_linear_y_(0.0),
  velocity_in_base_frame_angular_z_(0.0),
  sum_of_robot_center_projection_on_X_Y_axis_(0.0),
  wheels_radius_(0.0)
{
}

void Odometry::init(
  double time_sec, std::array<double, AUTONOMY_MECANUM_PLANAR_POINT_DIM> base_frame_offset)
{
  timestamp_ = time_sec;
  base_frame_offset_ = base_frame_offset;
}

bool Odometry::update(
  double wheel_front_left_vel, double wheel_rear_left_vel, double wheel_rear_right_vel,
  double wheel_front_right_vel, double dt)
{
  if (dt < 0.0001)
  {
    return false;
  }

  const double velocity_in_center_frame_linear_x =
    0.25 * wheels_radius_ *
    (wheel_front_left_vel + wheel_rear_left_vel + wheel_rear_right_vel + wheel_front_right_vel);
  const double velocity_in_center_frame_linear_y =
    0.25 * wheels_radius_ *
    (-wheel_front_left_vel + wheel_rear_left_vel - wheel_rear_right_vel + wheel_front_right_vel);
  const double velocity_in_center_frame_angular_z =
    0.25 * wheels_radius_ / sum_of_robot_center_projection_on_X_Y_axis_ *
    (-wheel_front_left_vel - wheel_rear_left_vel + wheel_rear_right_vel + wheel_front_right_vel);

  double velocity_in_center_wrt_base_x = 0.0;
  double velocity_in_center_wrt_base_y = 0.0;
  rotate2d(
    -base_frame_offset_[2], velocity_in_center_frame_linear_x, velocity_in_center_frame_linear_y,
    velocity_in_center_wrt_base_x, velocity_in_center_wrt_base_y);

  double linear_tf_x = 0.0;
  double linear_tf_y = 0.0;
  rotate2d(
    -base_frame_offset_[2], -base_frame_offset_[0], -base_frame_offset_[1], linear_tf_x,
    linear_tf_y);

  velocity_in_base_frame_linear_x_ =
    velocity_in_center_wrt_base_x + linear_tf_y * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_linear_y_ =
    velocity_in_center_wrt_base_y - linear_tf_x * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_angular_z_ = velocity_in_center_frame_angular_z;

  orientation_z_in_base_frame_ += velocity_in_base_frame_angular_z_ * dt;

  double velocity_in_odom_x = 0.0;
  double velocity_in_odom_y = 0.0;
  rotate2d(
    orientation_z_in_base_frame_, velocity_in_base_frame_linear_x_,
    velocity_in_base_frame_linear_y_, velocity_in_odom_x, velocity_in_odom_y);

  position_x_in_base_frame_ += velocity_in_odom_x * dt;
  position_y_in_base_frame_ += velocity_in_odom_y * dt;

  return true;
}

void Odometry::setWheelsParams(
  double sum_of_robot_center_projection_on_X_Y_axis, double wheels_radius)
{
  sum_of_robot_center_projection_on_X_Y_axis_ = sum_of_robot_center_projection_on_X_Y_axis;
  wheels_radius_ = wheels_radius;
}

}  // namespace mecanum_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy
