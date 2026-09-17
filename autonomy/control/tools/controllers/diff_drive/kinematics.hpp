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

// Ported into autonomy::control::tools::controllers
// Diff-drive inverse / forward kinematics (ROS-free).

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__KINEMATICS_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__KINEMATICS_HPP_

#include <utility>

#include "autonomy/control/tools/controllers/diff_drive/speed_limiter.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace diff_drive
{

/**
 * \brief Diff-drive wheel IK / FK.
 *
 * Inverse:
 *   left  = (v - ω * L/2) / r_left
 *   right = (v + ω * L/2) / r_right
 *
 * Forward:
 *   v = (r_right * ω_r + r_left * ω_l) / 2
 *   ω = (r_right * ω_r - r_left * ω_l) / L
 */
class DiffDriveKinematics
{
public:
  DiffDriveKinematics() = default;

  DiffDriveKinematics(
    double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    set_wheel_params(wheel_separation, left_wheel_radius, right_wheel_radius);
  }

  void set_wheel_params(
    double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    wheel_separation_ = wheel_separation;
    left_wheel_radius_ = left_wheel_radius;
    right_wheel_radius_ = right_wheel_radius;
  }

  void set_speed_limiter(SpeedLimiter * limiter) { speed_limiter_ = limiter; }

  double wheel_separation() const { return wheel_separation_; }
  double left_wheel_radius() const { return left_wheel_radius_; }
  double right_wheel_radius() const { return right_wheel_radius_; }

  /**
   * \brief Twist → wheel angular velocities [rad/s] (no speed limiting).
   */
  std::pair<double, double> inverse(double v, double omega) const
  {
    const double half_l = 0.5 * wheel_separation_;
    const double left = (v - omega * half_l) / left_wheel_radius_;
    const double right = (v + omega * half_l) / right_wheel_radius_;
    return {left, right};
  }

  /**
   * \brief Twist → wheel angular velocities with optional SpeedLimiter.
   *
   * When a limiter is attached and `dt > 0`, linear and angular commands are
   * limited independently via `SpeedLimiter::limit` before IK.
   */
  std::pair<double, double> inverse(
    double v, double omega, double v0, double v1, double omega0, double omega1, double dt)
  {
    if (speed_limiter_ && dt > 0.0)
    {
      speed_limiter_->limit(v, v0, v1, dt);
      speed_limiter_->limit(omega, omega0, omega1, dt);
    }
    return inverse(v, omega);
  }

  /** \brief Wheel angular velocities [rad/s] → body twist (v, ω). */
  std::pair<double, double> forward(double left_wheel_vel, double right_wheel_vel) const
  {
    const double left_lin = left_wheel_vel * left_wheel_radius_;
    const double right_lin = right_wheel_vel * right_wheel_radius_;
    const double v = 0.5 * (right_lin + left_lin);
    const double omega = (right_lin - left_lin) / wheel_separation_;
    return {v, omega};
  }

private:
  double wheel_separation_{0.0};
  double left_wheel_radius_{0.0};
  double right_wheel_radius_{0.0};
  SpeedLimiter * speed_limiter_{nullptr};
};

/** Free-function IK matching the classic diff-drive formula. */
inline std::pair<double, double> twist_to_wheel_velocities(
  double v, double omega, double wheel_separation, double left_wheel_radius,
  double right_wheel_radius)
{
  return DiffDriveKinematics(wheel_separation, left_wheel_radius, right_wheel_radius)
    .inverse(v, omega);
}

/** Free-function FK. */
inline std::pair<double, double> wheel_velocities_to_twist(
  double left_wheel_vel, double right_wheel_vel, double wheel_separation,
  double left_wheel_radius, double right_wheel_radius)
{
  return DiffDriveKinematics(wheel_separation, left_wheel_radius, right_wheel_radius)
    .forward(left_wheel_vel, right_wheel_vel);
}

}  // namespace diff_drive
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_DIFF_DRIVE__KINEMATICS_HPP_
