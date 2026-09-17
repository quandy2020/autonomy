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

/*
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

// Ported into autonomy::control::tools::controllers

#define _USE_MATH_DEFINES

#include "autonomy/control/tools/controllers/steering/steering_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace steering
{

SteeringKinematics::SteeringKinematics(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  steer_pos_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_track_traction_(0.0),
  wheel_track_steering_(0.0),
  wheel_base_(0.0),
  wheel_radius_(0.0),
  traction_wheel_old_pos_(0.0),
  traction_right_wheel_old_pos_(0.0),
  traction_left_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringKinematics::init(double time_sec)
{
  reset_accumulators();
  timestamp_ = time_sec;
}

bool SteeringKinematics::update_odometry(
  double linear_velocity, double angular_velocity, double dt)
{
  integrate_fk(linear_velocity, angular_velocity, dt);

  if (dt < 0.0001)
  {
    return false;
  }

  linear_acc_.accumulate(linear_velocity);
  angular_acc_.accumulate(angular_velocity);

  linear_ = linear_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

bool SteeringKinematics::update_from_position(
  double traction_wheel_pos, double steer_pos, double dt)
{
  const double traction_wheel_est_pos_diff = traction_wheel_pos - traction_wheel_old_pos_;
  traction_wheel_old_pos_ = traction_wheel_pos;
  return update_from_velocity(traction_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringKinematics::update_from_position(
  double traction_right_wheel_pos, double traction_left_wheel_pos, double steer_pos, double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringKinematics::update_from_position(
  double traction_right_wheel_pos, double traction_left_wheel_pos, double right_steer_pos,
  double left_steer_pos, double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, right_steer_pos,
    left_steer_pos, dt);
}

bool SteeringKinematics::update_from_velocity(
  double traction_wheel_vel, double steer_pos, double dt)
{
  steer_pos_ = steer_pos;
  const double linear_velocity = traction_wheel_vel * wheel_radius_;
  const double angular_velocity = std::tan(steer_pos) * linear_velocity / wheel_base_;
  return update_odometry(linear_velocity, angular_velocity, dt);
}

double SteeringKinematics::get_linear_velocity_double_traction_axle(
  double right_traction_wheel_vel, double left_traction_wheel_vel, double steer_pos)
{
  double turning_radius = wheel_base_ / std::tan(steer_pos);
  const double vel_wheel_r = right_traction_wheel_vel * wheel_radius_;
  const double vel_wheel_l = left_traction_wheel_vel * wheel_radius_;

  if (std::isinf(turning_radius))
  {
    return (vel_wheel_r + vel_wheel_l) * 0.5;
  }

  const double vel_r =
    vel_wheel_r * turning_radius / (turning_radius + wheel_track_traction_ * 0.5);
  const double vel_l =
    vel_wheel_l * turning_radius / (turning_radius - wheel_track_traction_ * 0.5);
  return (vel_r + vel_l) * 0.5;
}

bool SteeringKinematics::update_from_velocity(
  double right_traction_wheel_vel, double left_traction_wheel_vel, double steer_pos, double dt)
{
  steer_pos_ = steer_pos;
  const double linear_velocity = get_linear_velocity_double_traction_axle(
    right_traction_wheel_vel, left_traction_wheel_vel, steer_pos_);
  const double angular_velocity = std::tan(steer_pos_) * linear_velocity / wheel_base_;
  return update_odometry(linear_velocity, angular_velocity, dt);
}

bool SteeringKinematics::update_from_velocity(
  double right_traction_wheel_vel, double left_traction_wheel_vel, double right_steer_pos,
  double left_steer_pos, double dt)
{
  const double right_steer_pos_est = std::atan(
    wheel_base_ * std::tan(right_steer_pos) /
    (wheel_base_ - wheel_track_steering_ / 2 * std::tan(right_steer_pos)));
  const double left_steer_pos_est = std::atan(
    wheel_base_ * std::tan(left_steer_pos) /
    (wheel_base_ + wheel_track_steering_ / 2 * std::tan(left_steer_pos)));
  steer_pos_ = (right_steer_pos_est + left_steer_pos_est) * 0.5;

  const double linear_velocity = get_linear_velocity_double_traction_axle(
    right_traction_wheel_vel, left_traction_wheel_vel, steer_pos_);
  const double angular_velocity = std::tan(steer_pos_) * linear_velocity / wheel_base_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

void SteeringKinematics::update_open_loop(double v_bx, double omega_bz, double dt)
{
  linear_ = v_bx;
  angular_ = omega_bz;
  integrate_fk(v_bx, omega_bz, dt);
}

void SteeringKinematics::set_wheel_params(double wheel_radius, double wheel_base, double wheel_track)
{
  wheel_radius_ = wheel_radius;
  wheel_base_ = wheel_base;
  wheel_track_traction_ = wheel_track;
  wheel_track_steering_ = wheel_track;
}

void SteeringKinematics::set_wheel_params(
  double wheel_radius, double wheel_base, double wheel_track_steering, double wheel_track_traction)
{
  wheel_radius_ = wheel_radius;
  wheel_base_ = wheel_base;
  wheel_track_traction_ = wheel_track_traction;
  wheel_track_steering_ = wheel_track_steering;
}

void SteeringKinematics::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  reset_accumulators();
}

void SteeringKinematics::set_odometry_type(unsigned int type)
{
  config_type_ = static_cast<int>(type);
}

double SteeringKinematics::convert_twist_to_steering_angle(double v_bx, double omega_bz)
{
  const auto phi = std::atan(omega_bz * wheel_base_ / v_bx);
  return std::isfinite(phi) ? phi : 0.0;
}

std::tuple<std::vector<double>, std::vector<double>> SteeringKinematics::get_commands(
  double v_bx, double omega_bz, bool open_loop, bool reduce_wheel_speed_until_steering_reached)
{
  double Ws = 0.0;
  double phi = 0.0;
  double phi_IK = steer_pos_;

  phi = convert_twist_to_steering_angle(v_bx, omega_bz);
  if (open_loop)
  {
    phi_IK = phi;
  }
  Ws = v_bx / wheel_radius_;

  if (!open_loop && reduce_wheel_speed_until_steering_reached)
  {
    const double phi_delta = std::abs(steer_pos_ - phi);
    double scale = 1.0;
    const double min_phi_delta = M_PI / 6.;
    const double max_phi_delta = 1.5608;
    if (phi_delta >= min_phi_delta)
    {
      scale = std::cos(std::min(phi_delta, max_phi_delta)) / std::cos(min_phi_delta);
    }
    Ws *= scale;
  }

  if (config_type_ == static_cast<int>(BICYCLE_CONFIG))
  {
    return std::make_tuple(std::vector<double>{Ws}, std::vector<double>{phi});
  }
  if (config_type_ == static_cast<int>(TRICYCLE_CONFIG))
  {
    std::vector<double> traction_commands;
    if (is_close_to_zero(phi_IK))
    {
      traction_commands = {Ws, Ws};
    }
    else
    {
      const double turning_radius = wheel_base_ / std::tan(phi_IK);
      const double Wr = Ws * (turning_radius + wheel_track_traction_ * 0.5) / turning_radius;
      const double Wl = Ws * (turning_radius - wheel_track_traction_ * 0.5) / turning_radius;
      traction_commands = {Wr, Wl};
    }
    return std::make_tuple(traction_commands, std::vector<double>{phi});
  }
  if (config_type_ == static_cast<int>(ACKERMANN_CONFIG))
  {
    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    if (is_close_to_zero(phi_IK))
    {
      traction_commands = {Ws, Ws};
      steering_commands = {phi, phi};
    }
    else
    {
      const double turning_radius = wheel_base_ / std::tan(phi_IK);
      const double Wr = Ws * (turning_radius + wheel_track_traction_ * 0.5) / turning_radius;
      const double Wl = Ws * (turning_radius - wheel_track_traction_ * 0.5) / turning_radius;
      traction_commands = {Wr, Wl};

      const double numerator = 2 * wheel_base_ * std::sin(phi);
      const double denominator_first_member = 2 * wheel_base_ * std::cos(phi);
      const double denominator_second_member = wheel_track_steering_ * std::sin(phi);

      const double alpha_r =
        std::atan2(numerator, denominator_first_member + denominator_second_member);
      const double alpha_l =
        std::atan2(numerator, denominator_first_member - denominator_second_member);
      steering_commands = {alpha_r, alpha_l};
    }
    return std::make_tuple(traction_commands, steering_commands);
  }

  throw std::runtime_error("Config not implemented");
}

void SteeringKinematics::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  reset_accumulators();
}

void SteeringKinematics::integrate_runge_kutta_2(double v_bx, double omega_bz, double dt)
{
  const double theta_mid = heading_ + omega_bz * 0.5 * dt;
  x_ += v_bx * std::cos(theta_mid) * dt;
  y_ += v_bx * std::sin(theta_mid) * dt;
  heading_ += omega_bz * dt;
}

void SteeringKinematics::integrate_fk(double v_bx, double omega_bz, double dt)
{
  const double delta_x_b = v_bx * dt;
  const double delta_theta = omega_bz * dt;

  if (is_close_to_zero(delta_theta))
  {
    integrate_runge_kutta_2(v_bx, omega_bz, dt);
  }
  else
  {
    const double heading_old = heading_;
    const double R = delta_x_b / delta_theta;
    heading_ += delta_theta;
    x_ += R * (std::sin(heading_) - std::sin(heading_old));
    y_ += -R * (std::cos(heading_) - std::cos(heading_old));
  }
}

void SteeringKinematics::reset_accumulators()
{
  linear_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace steering
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy
