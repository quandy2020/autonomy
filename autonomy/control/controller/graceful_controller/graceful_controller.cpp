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

#include "autonomy/control/controller/graceful_controller/graceful_controller.hpp"

#include <cmath>
#include <memory>
#include <mutex>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/utils/controller_utils.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {

void GracefulController::Configure(const proto::ControllerOptions& options, std::string name,
                                   std::shared_ptr<transform::Buffer> tf,
                                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
  costmap_wrapper_ = costmap_wrapper;
  tf_buffer_ = tf;
  plugin_name_ = name;

  // Default transform tolerance
  double transform_tolerance = 0.1;

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(transform_tolerance, tf_buffer_, costmap_wrapper_);

  // Initialize footprint collision checker
  // TODO: Add params_ and collision detection support
  // if (params_->use_collision_detection) {
  //     collision_checker_ = std::make_unique<map::costmap_2d::
  //         FootprintCollisionChecker<map::costmap_2d::Costmap2D
  //         *>>(costmap_wrapper_->getCostmap());
  // }

  // Publishers - TODO: Need node parameter to create publishers
  // transformed_plan_pub_ =
  // node->CreateWriter<commsgs::planning_msgs::Path>("transformed_global_plan");
  // local_plan_pub_ =
  // node->CreateWriter<commsgs::planning_msgs::Path>("local_plan");
  // motion_target_pub_ =
  // node->CreateWriter<commsgs::geometry_msgs::PoseStamped>("motion_target");
  // slowdown_pub_ =
  // node->CreateWriter<commsgs::visualization_msgs::Marker>("slowdown");

  AINFO << "Configured Graceful Motion Controller: " << plugin_name_;
}

void GracefulController::Cleanup() {
  // Cleanup resources - reset smart pointers
  transformed_plan_pub_.reset();
  local_plan_pub_.reset();
  motion_target_pub_.reset();
  slowdown_pub_.reset();
  collision_checker_.reset();
  path_handler_.reset();
  control_law_.reset();
}

void GracefulController::Activate() {
  // Activate publishers if needed
  // Publishers are always active in autolink
}

void GracefulController::Deactivate() { AINFO << "Deactivating controller: " << plugin_name_; }

uint32 GracefulController::ComputeVelocityCommands(const commsgs::geometry_msgs::PoseStamped& pose,
                                                   const commsgs::geometry_msgs::TwistStamped& velocity,
                                                   commsgs::geometry_msgs::TwistStamped& cmd_vel,
                                                   common::GoalChecker* goal_checker, std::string& message) {
  // TODO: Add param_handler_ and params_ support
  // std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // Default parameters - TODO: Load from params_
  double max_robot_pose_search_dist = 3.0;
  double max_lookahead = 2.0;
  double min_lookahead = 0.3;
  double goal_dist_tolerance = 0.25;
  double in_place_collision_resolution = 0.1;
  double initial_rotation_tolerance = 0.1;
  bool use_collision_detection = false;
  bool prefer_final_rotation = false;
  bool allow_backward = false;
  bool initial_rotation = true;
  double k_phi = 1.0, k_delta = 1.0, beta = 1.0, lambda = 1.0;
  double slowdown_radius = 0.5;
  double v_linear_min = 0.0, v_linear_max = 0.5, v_angular_max = 1.0;
  double rotation_scaling_factor = 1.0;
  double v_angular_min_in_place = 0.1;

  // Update for the current goal checker's state
  commsgs::geometry_msgs::Pose pose_tolerance;
  commsgs::geometry_msgs::Twist velocity_tolerance;
  if (goal_checker && goal_checker->GetTolerances(pose_tolerance, velocity_tolerance)) {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  } else {
    goal_dist_tolerance_ = goal_dist_tolerance;
  }

  // Update the smooth control law with the new params
  // Initialize control_law_ if not already done
  if (!control_law_) {
    control_law_ = std::make_unique<SmoothControlLaw>(k_phi, k_delta, beta, lambda, slowdown_radius, v_linear_min,
                                                      v_linear_max, v_angular_max);
  } else {
    control_law_->SetCurvatureConstants(k_phi, k_delta, beta, lambda);
    control_law_->SetSlowdownRadius(slowdown_radius);
    control_law_->SetSpeedLimit(v_linear_min, v_linear_max, v_angular_max);
  }

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->TransformGlobalPlan(pose, max_robot_pose_search_dist);

  // Add proper orientations to plan, if needed
  ValidateOrientations(transformed_plan.poses);

  // Transform local frame to global frame to use in collision checking
  commsgs::geometry_msgs::TransformStamped costmap_transform;
  try {
    commsgs::builtin_interfaces::Time zero_time;
    zero_time.sec = 0;
    zero_time.nanosec = 0;
    costmap_transform = tf_buffer_->lookupTransform(costmap_wrapper_->getGlobalFrameID(),
                                                    costmap_wrapper_->getBaseFrameID(), zero_time, 0.1f);
  } catch (const std::exception& ex) {
    AERROR << "Could not transform " << costmap_wrapper_->getBaseFrameID() << " to "
           << costmap_wrapper_->getGlobalFrameID() << ": " << ex.what();
    message = "Transform error: " + std::string(ex.what());
    return 1;
  }

  // Compute distance to goal as the path's integrated distance to account for
  // path curvatures
  double dist_to_goal = 0.0;
  for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
    dist_to_goal +=
        map::costmap_2d::utils::euclidean_distance(transformed_plan.poses[i - 1], transformed_plan.poses[i]);
  }

  // If we've reached the XY goal tolerance, just rotate
  if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
    goal_reached_ = true;
    double angle_to_goal = transform::tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    // Check for collisions between our current pose and goal pose
    size_t num_steps = static_cast<size_t>(fabs(angle_to_goal) / in_place_collision_resolution);
    // Need to check at least the end pose
    num_steps = std::max(static_cast<size_t>(1), num_steps);
    bool collision_free = true;
    for (size_t i = 1; i <= num_steps; ++i) {
      double step = static_cast<double>(i) / static_cast<double>(num_steps);
      double yaw = step * angle_to_goal;
      commsgs::geometry_msgs::PoseStamped next_pose;
      next_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
      next_pose.pose.orientation = map::costmap_2d::utils::OrientationAroundZAxis(yaw);
      commsgs::geometry_msgs::PoseStamped costmap_pose;
      // Convert commsgs::TransformStamped to
      // geometry_msgs::TransformStamped for doTransform
      geometry_msgs::TransformStamped tf2_transform;
      tf2_transform.header.stamp = static_cast<uint64_t>(costmap_transform.header.stamp.sec) * 1000000000ULL +
                                   static_cast<uint64_t>(costmap_transform.header.stamp.nanosec);
      tf2_transform.header.frame_id = costmap_transform.header.frame_id;
      tf2_transform.child_frame_id = costmap_transform.child_frame_id;
      tf2_transform.transform.translation.x = costmap_transform.transform.translation.x;
      tf2_transform.transform.translation.y = costmap_transform.transform.translation.y;
      tf2_transform.transform.translation.z = costmap_transform.transform.translation.z;
      tf2_transform.transform.rotation.x = costmap_transform.transform.rotation.x;
      tf2_transform.transform.rotation.y = costmap_transform.transform.rotation.y;
      tf2_transform.transform.rotation.z = costmap_transform.transform.rotation.z;
      tf2_transform.transform.rotation.w = costmap_transform.transform.rotation.w;
      transform::tf2::doTransform(next_pose, costmap_pose, tf2_transform);
      if (use_collision_detection && InCollision(costmap_pose.pose.position.x, costmap_pose.pose.position.y,
                                                 transform::tf2::getYaw(costmap_pose.pose.orientation))) {
        collision_free = false;
        break;
      }
    }
    // Compute velocity if rotation is possible
    if (collision_free) {
      cmd_vel.twist = RotateToTarget(angle_to_goal);
      message = "";
      return 0;  // Return success code
    }
    // Else, fall through and see if we should follow control law longer
  }

  // Find a valid target pose and its trajectory
  commsgs::planning_msgs::Path local_plan;
  commsgs::geometry_msgs::PoseStamped target_pose;

  double dist_to_target;
  std::vector<double> target_distances;
  ComputeDistanceAlongPath(transformed_plan.poses, target_distances);

  bool is_first_iteration = true;
  for (int i = transformed_plan.poses.size() - 1; i >= 0; --i) {
    if (is_first_iteration) {
      // Calculate target pose through lookahead interpolation to get most
      // accurate lookahead point, if possible
      dist_to_target = max_lookahead;
      // Interpolate after goal false for graceful controller
      // Requires interpolating the orientation which is not yet
      // implemented Updates dist_to_target for target_pose returned if
      // using the point on the path
      target_pose = control::utils::GetLookAheadPoint(dist_to_target, transformed_plan, false);
      is_first_iteration = false;
    } else {
      // Underlying control law needs a single target pose, which should:
      //  * Be as far away as possible from the robot (for smoothness)
      //  * But no further than the max_lookahed_ distance
      //  * Be feasible to reach in a collision free manner
      dist_to_target = target_distances[i];
      target_pose = transformed_plan.poses[i];
    }

    // Compute velocity at this moment if valid target pose is found
    if (ValidateTargetPose(target_pose, dist_to_target, dist_to_goal, local_plan, costmap_transform, cmd_vel)) {
      // Publish the selected target_pose
      if (motion_target_pub_) {
        motion_target_pub_->Write(target_pose);
      }
      // Publish marker for slowdown radius around motion target for
      // debugging / visualization
      auto slowdown_marker = CreateSlowdownMarker(target_pose, slowdown_radius);
      if (slowdown_pub_) {
        slowdown_pub_->Write(slowdown_marker);
      }
      // Publish the local plan
      local_plan.header = transformed_plan.header;
      if (local_plan_pub_) {
        local_plan_pub_->Write(local_plan);
      }
      // Successfully found velocity command
      message = "";
      return 0;  // Return success code
    }
  }

  message = "Collision detected in trajectory";
  return 1;  // Return error code
}

bool GracefulController::IsGoalReached(double dist_tolerance, double angle_tolerance) {
  // TODO: Implement goal reached check based on current pose and goal
  // For now, return false as a placeholder
  (void)dist_tolerance;
  (void)angle_tolerance;
  return goal_reached_;
}

void GracefulController::SetPlan(const commsgs::planning_msgs::Path& path) {
  path_handler_->SetPlan(path);
  goal_reached_ = false;
  do_initial_rotation_ = true;
}

void GracefulController::SetSpeedLimit(const double& speed_limit, const bool& percentage) {
  // TODO: Add param_handler_ and params_ support
  // For now, update control_law_ directly if it exists
  if (control_law_) {
    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
      // Reset to default - TODO: Store initial values
      control_law_->SetSpeedLimit(0.0, 0.5, 1.0);
    } else {
      if (percentage) {
        // Speed limit is expressed in % from maximum speed of robot
        double v_linear_max = std::max(0.5 * speed_limit / 100.0, 0.0);
        double v_angular_max = 1.0 * speed_limit / 100.0;
        control_law_->SetSpeedLimit(0.0, v_linear_max, v_angular_max);
      } else {
        // Speed limit is expressed in m/s
        double v_linear_max = std::max(speed_limit, 0.0);
        // Limit the angular velocity to be proportional to the linear
        // velocity
        double v_angular_max = 1.0 * speed_limit / 0.5;
        control_law_->SetSpeedLimit(0.0, v_linear_max, v_angular_max);
      }
    }
  }
}

bool GracefulController::ValidateTargetPose(commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
                                            double dist_to_goal, commsgs::planning_msgs::Path& trajectory,
                                            commsgs::geometry_msgs::TransformStamped& costmap_transform,
                                            commsgs::geometry_msgs::TwistStamped& cmd_vel) {
  // Default parameters - TODO: Load from params_
  double max_lookahead = 2.0;
  double min_lookahead = 0.3;
  bool prefer_final_rotation = false;
  bool allow_backward = false;

  // Continue if target_pose is too far away from robot
  if (dist_to_target > max_lookahead) {
    return false;
  }

  if (dist_to_goal < max_lookahead) {
    if (prefer_final_rotation) {
      // Avoid instability and big sweeping turns at the end of paths by
      // ignoring final heading
      double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
      target_pose.pose.orientation = map::costmap_2d::utils::OrientationAroundZAxis(yaw);
    }
  } else if (dist_to_target < min_lookahead) {
    // Make sure target is far enough away to avoid instability
    return false;
  }

  // Flip the orientation of the motion target if the robot is moving
  // backwards
  bool reversing = false;
  if (allow_backward && target_pose.pose.position.x < 0.0) {
    reversing = true;
    target_pose.pose.orientation =
        map::costmap_2d::utils::OrientationAroundZAxis(transform::tf2::getYaw(target_pose.pose.orientation) + M_PI);
  }

  // Actually simulate the path
  if (SimulateTrajectory(target_pose, costmap_transform, trajectory, cmd_vel, reversing)) {
    // Successfully simulated to target_pose
    return true;
  }

  // Validation not successful
  return false;
}

bool GracefulController::SimulateTrajectory(const commsgs::geometry_msgs::PoseStamped& motion_target,
                                            const commsgs::geometry_msgs::TransformStamped& costmap_transform,
                                            commsgs::planning_msgs::Path& trajectory,
                                            commsgs::geometry_msgs::TwistStamped& cmd_vel, bool backward) {
  trajectory.poses.clear();

  // Default parameters - TODO: Load from params_
  bool initial_rotation = true;
  double initial_rotation_tolerance = 0.1;
  double v_linear_max = 0.5;
  bool use_collision_detection = false;

  // First pose is robot current pose
  commsgs::geometry_msgs::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;

  // Should we simulate rotation initially?
  bool sim_initial_rotation = do_initial_rotation_ && initial_rotation;
  double angle_to_target = std::atan2(motion_target.pose.position.y, motion_target.pose.position.x);
  if (fabs(angle_to_target) < initial_rotation_tolerance) {
    sim_initial_rotation = false;
    do_initial_rotation_ = false;
  }

  double distance = std::numeric_limits<double>::max();
  double resolution = costmap_wrapper_->getCostmap()->getResolution();
  double dt = (v_linear_max > 0.0) ? resolution / v_linear_max : 0.0;

  // Set max iter to avoid infinite loop
  unsigned int max_iter =
      3 *
      static_cast<unsigned int>(std::hypot(motion_target.pose.position.x, motion_target.pose.position.y) / resolution);

  // Generate path
  do {
    if (sim_initial_rotation) {
      // Compute rotation velocity
      double next_pose_yaw = transform::tf2::getYaw(next_pose.pose.orientation);
      auto cmd = RotateToTarget(angle_to_target - next_pose_yaw);

      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty()) {
        cmd_vel.twist = cmd;
      }

      // Are we done simulating initial rotation?
      if (fabs(angle_to_target - next_pose_yaw) < initial_rotation_tolerance) {
        sim_initial_rotation = false;
      }

      // Forward simulate rotation command
      next_pose_yaw += cmd_vel.twist.angular.z * dt;
      next_pose.pose.orientation = map::costmap_2d::utils::OrientationAroundZAxis(next_pose_yaw);
    } else {
      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty() && control_law_) {
        cmd_vel.twist = control_law_->CalculateRegularVelocity(motion_target.pose, next_pose.pose, backward);
      }

      // Apply velocities to calculate next pose
      if (control_law_) {
        next_pose.pose = control_law_->CalculateNextPose(dt, motion_target.pose, next_pose.pose, backward);
      }
    }

    // Add the pose to the trajectory for visualization
    trajectory.poses.push_back(next_pose);

    // Check for collision
    commsgs::geometry_msgs::PoseStamped global_pose;
    // Convert commsgs::TransformStamped to geometry_msgs::TransformStamped
    // for doTransform
    geometry_msgs::TransformStamped tf2_transform;
    tf2_transform.header.stamp = static_cast<uint64_t>(costmap_transform.header.stamp.sec) * 1000000000ULL +
                                 static_cast<uint64_t>(costmap_transform.header.stamp.nanosec);
    tf2_transform.header.frame_id = costmap_transform.header.frame_id;
    tf2_transform.child_frame_id = costmap_transform.child_frame_id;
    tf2_transform.transform.translation.x = costmap_transform.transform.translation.x;
    tf2_transform.transform.translation.y = costmap_transform.transform.translation.y;
    tf2_transform.transform.translation.z = costmap_transform.transform.translation.z;
    tf2_transform.transform.rotation.x = costmap_transform.transform.rotation.x;
    tf2_transform.transform.rotation.y = costmap_transform.transform.rotation.y;
    tf2_transform.transform.rotation.z = costmap_transform.transform.rotation.z;
    tf2_transform.transform.rotation.w = costmap_transform.transform.rotation.w;
    transform::tf2::doTransform(next_pose, global_pose, tf2_transform);
    if (use_collision_detection && InCollision(global_pose.pose.position.x, global_pose.pose.position.y,
                                               transform::tf2::getYaw(global_pose.pose.orientation))) {
      return false;
    }

    // Check if we reach the goal
    distance = map::costmap_2d::utils::euclidean_distance(motion_target.pose, next_pose.pose);
  } while (distance > resolution && trajectory.poses.size() < max_iter);

  return true;
}

commsgs::geometry_msgs::Twist GracefulController::RotateToTarget(double angle_to_target) {
  // Default parameters - TODO: Load from params_
  double rotation_scaling_factor = 1.0;
  double v_angular_max = 1.0;
  double v_angular_min_in_place = 0.1;

  commsgs::geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = rotation_scaling_factor * angle_to_target * v_angular_max;
  vel.angular.z = std::copysign(1.0, vel.angular.z) *
                  std::max(static_cast<double>(std::abs(vel.angular.z)), v_angular_min_in_place);
  return vel;
}

bool GracefulController::InCollision(const double& x, const double& y, const double& theta) {
  if (!collision_checker_) {
    return false;  // No collision checker available
  }

  unsigned int mx, my;
  if (!costmap_wrapper_->getCostmap()->worldToMap(x, y, mx, my)) {
    AWARN << "The path is not in the costmap. Cannot check for collisions. "
          << "Proceed at your own risk, slow the robot, or increase your "
             "costmap size.";
    return false;
  }

  // Calculate the cost of the footprint at the robot's current position
  // depending on the shape of the footprint
  // TODO: Add support for isTrackingUnknown and getUseRadius
  bool is_tracking_unknown = false;
  bool consider_footprint = true;  // Default to footprint checking

  double footprint_cost;
  if (consider_footprint) {
    // TODO: Get robot footprint from costmap_wrapper_
    std::vector<commsgs::geometry_msgs::Point> footprint;
    footprint_cost = collision_checker_->footprintCostAtPose(x, y, theta, footprint);
  } else {
    footprint_cost = collision_checker_->pointCost(mx, my);
  }

  switch (static_cast<unsigned char>(footprint_cost)) {
    case (map::costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint ? false : true;
    case (map::costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

void GracefulController::ComputeDistanceAlongPath(const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
                                                  std::vector<double>& distances) {
  distances.resize(poses.size());
  // Do the first pose from robot
  double d = std::hypot(poses[0].pose.position.x, poses[0].pose.position.y);
  distances[0] = d;
  // Compute remaining poses
  for (size_t i = 1; i < poses.size(); ++i) {
    d += map::costmap_2d::utils::euclidean_distance(poses[i - 1], poses[i]);
    distances[i] = d;
  }
}

void GracefulController::ValidateOrientations(std::vector<commsgs::geometry_msgs::PoseStamped>& path) {
  // We never change the orientation of the first & last pose
  // So we need at least three poses to do anything here
  if (path.size() < 3) {
    return;
  }

  // Check if we actually need to add orientations
  double initial_yaw = transform::tf2::getYaw(path[1].pose.orientation);
  for (size_t i = 2; i < path.size() - 1; ++i) {
    double this_yaw = transform::tf2::getYaw(path[i].pose.orientation);
    if (std::abs(autonomy::common::math::AngleDiff(this_yaw, initial_yaw)) > 1e-6) {
      return;
    }
  }

  // For each pose, point at the next one
  // NOTE: control loop will handle reversing logic
  for (size_t i = 0; i < path.size() - 1; ++i) {
    // Get relative yaw angle
    double dx = path[i + 1].pose.position.x - path[i].pose.position.x;
    double dy = path[i + 1].pose.position.y - path[i].pose.position.y;
    double yaw = std::atan2(dy, dx);
    path[i].pose.orientation = map::costmap_2d::utils::OrientationAroundZAxis(yaw);
  }
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::GracefulController,
                            autonomy::control::common::ControllerInterface)