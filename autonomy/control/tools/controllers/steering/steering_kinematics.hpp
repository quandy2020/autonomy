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
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

// Ported into autonomy::control::tools::controllers

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__STEERING_KINEMATICS_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__STEERING_KINEMATICS_HPP_

#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include "autonomy/control/tools/rolling_mean.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace steering
{

constexpr unsigned int BICYCLE_CONFIG = 0;
constexpr unsigned int TRICYCLE_CONFIG = 1;
constexpr unsigned int ACKERMANN_CONFIG = 2;

inline bool is_close_to_zero(double val) { return std::fabs(val) < 1e-6; }

class SteeringKinematics
{
public:
  explicit SteeringKinematics(size_t velocity_rolling_window_size = 10);

  void init(double time_sec = 0.0);

  bool update_from_position(
    double traction_wheel_pos, double steer_pos, double dt);

  bool update_from_position(
    double right_traction_wheel_pos, double left_traction_wheel_pos, double steer_pos, double dt);

  bool update_from_position(
    double right_traction_wheel_pos, double left_traction_wheel_pos, double right_steer_pos,
    double left_steer_pos, double dt);

  bool update_from_velocity(double traction_wheel_vel, double steer_pos, double dt);

  bool update_from_velocity(
    double right_traction_wheel_vel, double left_traction_wheel_vel, double steer_pos, double dt);

  bool update_from_velocity(
    double right_traction_wheel_vel, double left_traction_wheel_vel, double right_steer_pos,
    double left_steer_pos, double dt);

  void update_open_loop(double v_bx, double omega_bz, double dt);

  void set_odometry_type(unsigned int type);

  unsigned int get_odometry_type() const { return static_cast<unsigned int>(config_type_); }

  double get_heading() const { return heading_; }
  double get_x() const { return x_; }
  double get_y() const { return y_; }
  double get_linear() const { return linear_; }
  double get_angular() const { return angular_; }

  void set_wheel_params(double wheel_radius, double wheel_base = 0.0, double wheel_track = 0.0);

  void set_wheel_params(
    double wheel_radius, double wheel_base, double wheel_track_steering,
    double wheel_track_traction);

  void set_velocity_rolling_window_size(size_t velocity_rolling_window_size);

  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    double v_bx, double omega_bz, bool open_loop = true,
    bool reduce_wheel_speed_until_steering_reached = false);

  void reset_odometry();

private:
  bool update_odometry(double v_bx, double omega_bz, double dt);
  void integrate_runge_kutta_2(double v_bx, double omega_bz, double dt);
  void integrate_fk(double v_bx, double omega_bz, double dt);
  double convert_twist_to_steering_angle(double v_bx, double omega_bz);
  double get_linear_velocity_double_traction_axle(
    double right_traction_wheel_vel, double left_traction_wheel_vel, double steer_pos);
  void reset_accumulators();

  using RollingMeanAccumulator = autonomy::control::tools::RollingMeanAccumulator<double>;

  double timestamp_;

  double x_;
  double y_;
  double steer_pos_;
  double heading_;

  double linear_;
  double angular_;

  double wheel_track_traction_;
  double wheel_track_steering_;
  double wheel_base_;
  double wheel_radius_;

  int config_type_ = -1;

  double traction_wheel_old_pos_;
  double traction_right_wheel_old_pos_;
  double traction_left_wheel_old_pos_;
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_acc_;
  RollingMeanAccumulator angular_acc_;
};

}  // namespace steering
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_STEERING__STEERING_KINEMATICS_HPP_
