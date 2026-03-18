/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace control {
namespace controller {

/**
 * @class nav2_graceful_controller::SmoothControlLaw
 * @brief Smooth control law for graceful motion based on "A smooth control law
 * for graceful motion" (Jong Jin Park and Benjamin Kuipers).
 */
class SmoothControlLaw {
 public:
  /**
   * @brief Constructor for nav2_graceful_controller::SmoothControlLaw
   *
   * @param k_phi Ratio of the rate of change in phi to the rate of change in
   * r.
   * @param k_delta Constant factor applied to the heading error feedback.
   * @param beta Constant factor applied to the path curvature: dropping
   * velocity.
   * @param lambda Constant factor applied to the path curvature for
   * sharpness.
   * @param slowdown_radius Radial threshold applied to the slowdown rule.
   * @param v_linear_min Minimum linear velocity.
   * @param v_linear_max Maximum linear velocity.
   * @param v_angular_max Maximum angular velocity.
   */
  SmoothControlLaw(double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
                   double v_linear_min, double v_linear_max, double v_angular_max);

  /**
   * @brief Destructor for nav2_graceful_controller::SmoothControlLaw
   */
  ~SmoothControlLaw() = default;

  /**
   * @brief Set the values that define the curvature.
   *
   * @param k_phi Ratio of the rate of change in phi to the rate of change in
   * r.
   * @param k_delta Constant factor applied to the heading error feedback.
   * @param beta Constant factor applied to the path curvature: dropping
   * velocity.
   * @param lambda Constant factor applied to the path curvature for
   * sharpness.
   */
  void SetCurvatureConstants(const double k_phi, const double k_delta, const double beta, const double lambda);

  /**
   * @brief Set the slowdown radius
   *
   * @param slowdown_radius Radial threshold applied to the slowdown rule.
   */
  void SetSlowdownRadius(const double slowdown_radius);

  /**
   * @brief Update the velocity limits.
   *
   * @param v_linear_min The minimum absolute velocity in the linear
   * direction.
   * @param v_linear_max The maximum absolute velocity in the linear
   * direction.
   * @param v_angular_max The maximum absolute velocity in the angular
   * direction.
   */
  void SetSpeedLimit(const double v_linear_min, const double v_linear_max, const double v_angular_max);

  /**
   * @brief Compute linear and angular velocities command using the curvature.
   *
   * @param target Pose of the target in the robot frame.
   * @param current Current pose of the robot in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to
   * false.
   * @return Velocity command.
   */
  commsgs::geometry_msgs::Twist CalculateRegularVelocity(const commsgs::geometry_msgs::Pose& target,
                                                         const commsgs::geometry_msgs::Pose& current,
                                                         const bool& backward = false);

  /**
   * @brief Compute linear and angular velocities command using the curvature.
   *
   * @param target Pose of the target in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to
   * false.
   * @return Velocity command.
   */
  commsgs::geometry_msgs::Twist CalculateRegularVelocity(const commsgs::geometry_msgs::Pose& target,
                                                         const bool& backward = false);

  /**
   * @brief Calculate the next pose of the robot by generating a velocity
   * command using the curvature and the current pose.
   *
   * @param dt Time step.
   * @param target Pose of the target in the robot frame.
   * @param current Current pose of the robot in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to
   * false.
   * @return commsgs::geometry_msgs::Pose
   */
  commsgs::geometry_msgs::Pose CalculateNextPose(const double dt, const commsgs::geometry_msgs::Pose& target,
                                                 const commsgs::geometry_msgs::Pose& current,
                                                 const bool& backward = false);

 protected:
  /**
   * @brief Calculate the path curvature using a Lyapunov-based feedback
   * control law from "A smooth control law for graceful motion" (Jong Jin
   * Park and Benjamin Kuipers).
   *
   * @param r Distance between the robot and the target.
   * @param phi Orientation of target with respect to the line of sight from
   * the robot to the target.
   * @param delta Steering angle of the robot.
   * @return The curvature
   */
  double CalculateCurvature(double r, double phi, double delta);

  /**
   * @brief Ratio of the rate of change in phi to the rate of change in r.
   * Controls the convergence of the slow subsystem.
   *
   * If this value is equal to zero, the controller will behave as a pure
   * waypoint follower. A high value offers extreme scenario of pose-following
   * where theta is reduced much faster than r.
   *
   * Note: This variable is called k1 in earlier versions of the paper.
   */
  double k_phi_;

  /**
   * @brief Constant factor applied to the heading error feedback. Controls
   * the convergence of the fast subsystem.
   *
   * The bigger the value, the robot converge faster to the reference heading.
   *
   * Note: This variable is called k2 in earlier versions of the paper.
   */
  double k_delta_;

  /**
   * @brief Constant factor applied to the path curvature. This value must be
   * positive. Determines how fast the velocity drops when the curvature
   * increases.
   */
  double beta_;

  /**
   * @brief Constant factor applied to the path curvature. This value must be
   * greater or equal to 1. Determines the sharpness of the curve: higher
   * lambda implies sharper curves.
   */
  double lambda_;

  /**
   * @brief Radial threshold applied to the slowdown rule.
   */
  double slowdown_radius_;

  /**
   * @brief Minimum linear velocity.
   */
  double v_linear_min_;

  /**
   * @brief Maximum linear velocity.
   */
  double v_linear_max_;

  /**
   * @brief Maximum angular velocity.
   */
  double v_angular_max_;
};

}  // namespace controller
}  // namespace control
}  // namespace autonomy