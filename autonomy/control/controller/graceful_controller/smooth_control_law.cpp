/*
 * Copyright 2025 The Openbot Authors
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

#include "autonomy/control/controller/graceful_controller/smooth_control_law.hpp"

#include "autonomy/control/controller/graceful_controller/ego_polar_coords.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {

SmoothControlLaw::SmoothControlLaw(double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
                                   double v_linear_min, double v_linear_max, double v_angular_max)
    : k_phi_(k_phi),
      k_delta_(k_delta),
      beta_(beta),
      lambda_(lambda),
      slowdown_radius_(slowdown_radius),
      v_linear_min_(v_linear_min),
      v_linear_max_(v_linear_max),
      v_angular_max_(v_angular_max) {}

void SmoothControlLaw::SetCurvatureConstants(double k_phi, double k_delta, double beta, double lambda) {
  k_phi_ = k_phi;
  k_delta_ = k_delta;
  beta_ = beta;
  lambda_ = lambda;
}

void SmoothControlLaw::SetSlowdownRadius(double slowdown_radius) { slowdown_radius_ = slowdown_radius; }

void SmoothControlLaw::SetSpeedLimit(const double v_linear_min, const double v_linear_max, const double v_angular_max) {
  v_linear_min_ = v_linear_min;
  v_linear_max_ = v_linear_max;
  v_angular_max_ = v_angular_max;
}

commsgs::geometry_msgs::Twist SmoothControlLaw::CalculateRegularVelocity(const commsgs::geometry_msgs::Pose& target,
                                                                         const commsgs::geometry_msgs::Pose& current,
                                                                         const bool& backward) {
  // Convert the target to polar coordinates
  auto ego_coords = EgocentricPolarCoordinates(target, current, backward);
  // Calculate the curvature
  double curvature = CalculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);
  // Invert the curvature if the robot is moving backwards
  curvature = backward ? -curvature : curvature;

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  double v = v_linear_max_ / (1.0 + beta_ * std::pow(fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  v = std::min(v_linear_max_ * (ego_coords.r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  v = std::clamp(v, v_linear_min_, v_linear_max_);

  // Set the velocity to negative if the robot is moving backwards
  v = backward ? -v : v;

  // Compute the angular velocity
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  // Return the velocity command
  commsgs::geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w_bound;
  return cmd_vel;
}

commsgs::geometry_msgs::Twist SmoothControlLaw::CalculateRegularVelocity(const commsgs::geometry_msgs::Pose& target,
                                                                         const bool& backward) {
  return CalculateRegularVelocity(target, commsgs::geometry_msgs::Pose(), backward);
}

commsgs::geometry_msgs::Pose SmoothControlLaw::CalculateNextPose(const double dt,
                                                                 const commsgs::geometry_msgs::Pose& target,
                                                                 const commsgs::geometry_msgs::Pose& current,
                                                                 const bool& backward) {
  commsgs::geometry_msgs::Twist vel = CalculateRegularVelocity(target, current, backward);
  commsgs::geometry_msgs::Pose next;
  double yaw = transform::tf2::getYaw(current.orientation);
  next.position.x = current.position.x + vel.linear.x * dt * cos(yaw);
  next.position.y = current.position.y + vel.linear.x * dt * sin(yaw);
  yaw += vel.angular.z * dt;
  next.orientation = map::costmap_2d::utils::OrientationAroundZAxis(yaw);
  return next;
}

double SmoothControlLaw::CalculateCurvature(double r, double phi, double delta) {
  // Calculate the proportional term of the control law as the product of the
  // gain and the error: difference between the actual steering angle and the
  // virtual control for the slow subsystem
  double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  double feedback_term = (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * phi, 2)))) * sin(delta);
  // Calculate the path curvature
  return -1.0 / r * (prop_term + feedback_term);
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy