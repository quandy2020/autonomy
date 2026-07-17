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

#include "autonomy/control/controller/pure_pursuit_controller/controller.hpp"

#include <cmath>
#include <memory>
#include <mutex>

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace pure_pursuit_controller {

void RegulatedPurePursuitController::Configure(const proto::ControllerOptions& options, std::string name,
                                               std::shared_ptr<transform::Buffer> tf_buffer,
                                               std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
  costmap_wrapper_ = costmap_wrapper;
  if (!costmap_wrapper_) {
    throw autonomy::control::common::ControllerException("RegulatedPurePursuitController: costmap_wrapper is null");
  }
  costmap_ = costmap_wrapper_->getCostmap();
  if (!costmap_) {
    throw autonomy::control::common::ControllerException("RegulatedPurePursuitController: costmap is null");
  }
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;

  // Handles storage and dynamic configuration of parameters using proto.
  param_handler_ =
      std::make_unique<ParameterHandler>(options.pure_pursuit_controller_options(), costmap_->getSizeInMetersX());
  pp_options_ = &param_handler_->GetOptions();

  // Handles global path transformations
  if (!tf_buffer_) {
    AWARN << "RegulatedPurePursuitController configured without TF buffer; controller will be inactive until TF is "
             "provided.";
    path_handler_.reset();
  } else {
    path_handler_ = std::make_unique<PathHandler>(pp_options_->transform_tolerance(), tf_buffer_, costmap_wrapper_);
  }

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node_, costmap_wrapper_, pp_options_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update
  control_duration_ = 1.0 / control_frequency;

  // Publishers - TODO: Need node parameter to create publishers
  // global_path_pub_ = node->CreateWriter<commsgs::planning_msgs::Path>("received_global_plan");
  // carrot_pub_ = node->CreateWriter<commsgs::geometry_msgs::PointStamped>("lookahead_point");
  // curvature_carrot_pub_ = node->CreateWriter<commsgs::geometry_msgs::PointStamped>("curvature_lookahead_point");

  AINFO << "Configured Regulated Pure Pursuit Controller: " << plugin_name_;
}

void RegulatedPurePursuitController::Cleanup() {
  AINFO << "Cleaning up controller: " << plugin_name_;
  global_path_pub_.reset();
  carrot_pub_.reset();
  curvature_carrot_pub_.reset();
}

void RegulatedPurePursuitController::Activate() {
  AINFO << "Activating controller: " << plugin_name_;
  // Publishers are always active in autolink
}

void RegulatedPurePursuitController::Deactivate() { AINFO << "Deactivating controller: " << plugin_name_; }

std::unique_ptr<commsgs::geometry_msgs::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
    const commsgs::geometry_msgs::PoseStamped& carrot_pose) {
  auto carrot_msg = std::make_unique<commsgs::geometry_msgs::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(const commsgs::geometry_msgs::TwistStamped& speed) {
  if (!pp_options_) {
    return 1.0;
  }
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  if (pp_options_->use_velocity_scaled_lookahead_dist()) {
    const double lookahead_dist = fabs(speed.twist.linear.x) * pp_options_->lookahead_time();
    return std::clamp(lookahead_dist, pp_options_->min_lookahead_dist(), pp_options_->max_lookahead_dist());
  }
  return pp_options_->lookahead_dist();
}

double calculateCurvature(commsgs::geometry_msgs::Point lookahead_point) {
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 = (lookahead_point.x * lookahead_point.x) + (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

uint32 RegulatedPurePursuitController::ComputeVelocityCommands(const commsgs::geometry_msgs::PoseStamped& pose,
                                                               const commsgs::geometry_msgs::TwistStamped& speed,
                                                               commsgs::geometry_msgs::TwistStamped& cmd_vel,
                                                               common::GoalChecker* goal_checker,
                                                               std::string& message) {
  // Default output
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.linear.z = 0.0;
  cmd_vel.twist.angular.x = 0.0;
  cmd_vel.twist.angular.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (!path_handler_) {
    message = "RegulatedPurePursuitController not initialized (missing TF buffer / path handler)";
    return proto::CONTROLLER_RESULT_NOT_INITIALIZED;
  }
  if (!pp_options_ || !param_handler_) {
    message = "RegulatedPurePursuitController not initialized (missing parameters)";
    return proto::CONTROLLER_RESULT_NOT_INITIALIZED;
  }

  if (param_handler_) {
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  }

  map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
  std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  commsgs::geometry_msgs::Pose pose_tolerance;
  commsgs::geometry_msgs::Twist velocity_tolerance;
  if (goal_checker && goal_checker->GetTolerances(pose_tolerance, velocity_tolerance)) {
    goal_dist_tol_ = pose_tolerance.position.x;
  } else {
    goal_dist_tol_ = 0.25;  // Default
  }

  // Transform path to robot base frame
  const double max_robot_pose_search_dist = pp_options_->max_robot_pose_search_dist();
  const bool interpolate_curvature_after_goal = pp_options_->interpolate_curvature_after_goal();
  auto transformed_plan =
      path_handler_->transformGlobalPlan(pose, max_robot_pose_search_dist, interpolate_curvature_after_goal);
  if (global_path_pub_) {
    global_path_pub_->Write(transformed_plan);
  }

  // Cache goal info for IsGoalReached()
  last_dist_to_goal_ =
      std::hypot(transformed_plan.poses.back().pose.position.x, transformed_plan.poses.back().pose.position.y);
  last_angle_to_goal_ = transform::tf2::getYaw(transformed_plan.poses.back().pose.orientation);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);
  double curv_lookahead_dist = pp_options_->curvature_lookahead_dist();

  // Check for reverse driving
  const bool allow_reversing = pp_options_->allow_reversing();
  if (allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
    if (dist_to_cusp < curv_lookahead_dist) {
      curv_lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  if (carrot_pub_) {
    carrot_pub_->Write(*createCarrotMsg(carrot_pose));
  }

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  const bool use_fixed_curvature_lookahead = pp_options_->use_fixed_curvature_lookahead();
  if (use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose =
        getLookAheadPoint(curv_lookahead_dist, transformed_plan, interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    if (curvature_carrot_pub_) {
      curvature_carrot_pub_->Write(*createCarrotMsg(curvature_lookahead_pose));
    }
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = pp_options_->desired_linear_vel();

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = transform::tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    is_rotating_to_heading_ = false;
    double pose_cost = 0.0;
    if (collision_checker_) {
      pose_cost = collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y);
    }
    applyConstraints(regulation_curvature, speed, pose_cost, transformed_plan, linear_vel, x_vel_sign);

    if (cancelling_ && pp_options_) {
      const double& dt = control_duration_;
      const double cancel_deceleration = pp_options_->cancel_deceleration();
      linear_vel = speed.twist.linear.x - x_vel_sign * dt * cancel_deceleration;

      if (x_vel_sign > 0) {
        if (linear_vel <= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      } else {
        if (linear_vel >= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      }
    }

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * regulation_curvature;
  }

  // Collision checking on this velocity heading
  const double& carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  const bool use_collision_detection = pp_options_->use_collision_detection();
  if (use_collision_detection && collision_checker_) {
    if (collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
      throw autonomy::control::common::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    }
  }

  // Publish whether we are rotating to goal heading
  // TODO: Add Bool message type to commsgs or use a different approach
  // if (is_rotating_to_heading_pub_) {
  //     commsgs::std_msgs::Bool is_rotating_to_heading_msg;
  //     is_rotating_to_heading_msg.data = is_rotating_to_heading_;
  //     is_rotating_to_heading_pub_->Write(is_rotating_to_heading_msg);
  // }

  // populate and return message
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  message = "";  // Success
  return proto::CONTROLLER_RESULT_SUCCESS;
}

bool RegulatedPurePursuitController::shouldRotateToPath(const commsgs::geometry_msgs::PoseStamped& carrot_pose,
                                                        double& angle_to_path, double& x_vel_sign) {
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  // In case we are reversing
  if (x_vel_sign < 0.0) {
    // Normalize angle to [-pi, pi]
    angle_to_path = std::fmod(angle_to_path + M_PI + M_PI, 2.0 * M_PI) - M_PI;
  }
  const bool use_rotate_to_heading = pp_options_ ? pp_options_->use_rotate_to_heading() : false;
  const double rotate_to_heading_min_angle = pp_options_ ? pp_options_->rotate_to_heading_min_angle() : 0.1;
  return use_rotate_to_heading && fabs(angle_to_path) > rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(const commsgs::geometry_msgs::PoseStamped& carrot_pose) {
  // Whether we should rotate robot to goal heading
  const bool use_rotate_to_heading = pp_options_ ? pp_options_->use_rotate_to_heading() : false;
  if (!use_rotate_to_heading) {
    return false;
  }

  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);

  const bool stateful = pp_options_ ? pp_options_->stateful() : false;
  if (stateful) {
    if (!has_reached_xy_tolerance_ && dist_to_goal < goal_dist_tol_) {
      has_reached_xy_tolerance_ = true;
    }
    return has_reached_xy_tolerance_;
  }

  return dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(double& linear_vel, double& angular_vel,
                                                     const double& angle_to_path,
                                                     const commsgs::geometry_msgs::TwistStamped& curr_speed) {
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  const double rotate_to_heading_angular_vel = pp_options_ ? pp_options_->rotate_to_heading_angular_vel() : 1.0;
  angular_vel = sign * rotate_to_heading_angular_vel;

  const double& dt = control_duration_;
  const double max_angular_accel = pp_options_ ? pp_options_->max_angular_accel() : 1.0;
  const double min_feasible_angular_speed = curr_speed.twist.angular.z - max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.twist.angular.z + max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

commsgs::geometry_msgs::Point RegulatedPurePursuitController::circleSegmentIntersection(
    const commsgs::geometry_msgs::Point& p1, const commsgs::geometry_msgs::Point& p2, double r) {
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  commsgs::geometry_msgs::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

commsgs::geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
    const double& lookahead_dist, const commsgs::planning_msgs::Path& transformed_plan, bool interpolate_after_goal) {
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto& ps) {
    return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
  });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    if (interpolate_after_goal) {
      auto last_pose_it = std::prev(transformed_plan.poses.end());
      auto prev_last_pose_it = std::prev(last_pose_it);

      double end_path_orientation = atan2(last_pose_it->pose.position.y - prev_last_pose_it->pose.position.y,
                                          last_pose_it->pose.position.x - prev_last_pose_it->pose.position.x);

      // Project the last segment out to guarantee it is beyond the look ahead
      // distance
      auto projected_position = last_pose_it->pose.position;
      projected_position.x += cos(end_path_orientation) * lookahead_dist;
      projected_position.y += sin(end_path_orientation) * lookahead_dist;

      // Use the circle intersection to find the position at the correct look
      // ahead distance
      const auto interpolated_position =
          circleSegmentIntersection(last_pose_it->pose.position, projected_position, lookahead_dist);

      commsgs::geometry_msgs::PoseStamped interpolated_pose;
      interpolated_pose.header = last_pose_it->header;
      interpolated_pose.pose.position = interpolated_position;
      return interpolated_pose;
    } else {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
  } else if (goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

void RegulatedPurePursuitController::applyConstraints(const double& curvature,
                                                      const commsgs::geometry_msgs::TwistStamped& /*curr_speed*/,
                                                      const double& pose_cost, const commsgs::planning_msgs::Path& path,
                                                      double& linear_vel, double& sign) {
  if (!pp_options_) {
    return;
  }

  double curvature_vel = linear_vel, cost_vel = linear_vel;

  // limit the linear velocity by curvature
  if (pp_options_->use_regulated_linear_velocity_scaling()) {
    curvature_vel =
        heuristics::curvatureConstraint(linear_vel, curvature, pp_options_->regulated_linear_scaling_min_radius());
  }

  // limit the linear velocity by proximity to obstacles
  if (pp_options_->use_cost_regulated_linear_velocity_scaling()) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_wrapper_, *pp_options_);
  }

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, pp_options_->regulated_linear_scaling_min_speed());

  // Apply constraint to reduce speed on approach to the final goal pose
  linear_vel = heuristics::approachVelocityConstraint(linear_vel, path, pp_options_->min_approach_linear_velocity(),
                                                      pp_options_->approach_velocity_scaling_dist());

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, pp_options_->desired_linear_vel());
  linear_vel = sign * linear_vel;
}

bool RegulatedPurePursuitController::IsGoalReached(double dist_tolerance, double angle_tolerance) {
  if (!std::isfinite(last_dist_to_goal_) || !std::isfinite(last_angle_to_goal_)) {
    return false;
  }
  return (last_dist_to_goal_ <= dist_tolerance) && (std::fabs(last_angle_to_goal_) <= angle_tolerance);
}

void RegulatedPurePursuitController::SetPlan(const commsgs::planning_msgs::Path& path) {
  has_reached_xy_tolerance_ = false;
  path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::SetSpeedLimit(const double& speed_limit, const bool& percentage) {
  if (!pp_options_ || !param_handler_) {
    return;
  }

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    param_handler_->RestoreBaseDesiredLinearVel();
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      param_handler_->SetDesiredLinearVel(param_handler_->GetBaseDesiredLinearVel() * speed_limit / 100.0);
    } else {
      // Speed limit is expressed in absolute value
      param_handler_->SetDesiredLinearVel(speed_limit);
    }
  }
}

void RegulatedPurePursuitController::Reset() {
  cancelling_ = false;
  finished_cancelling_ = false;
  has_reached_xy_tolerance_ = false;
}

bool RegulatedPurePursuitController::cancel() {
  // Legacy method for compatibility
  if (!pp_options_ || !pp_options_->use_cancel_deceleration()) {
    return true;
  }
  cancelling_ = true;
  return finished_cancelling_;
}

double RegulatedPurePursuitController::findVelocitySignChange(const commsgs::planning_msgs::Path& transformed_plan) {
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x - transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y - transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x - transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y - transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existence of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_prod < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return std::hypot(transformed_plan.poses[pose_id].pose.position.x,
                        transformed_plan.poses[pose_id].pose.position.y);
    }

    // Check if orientations differ using yaw angles
    double yaw1 = transform::tf2::getYaw(transformed_plan.poses[pose_id - 1].pose.orientation);
    double yaw2 = transform::tf2::getYaw(transformed_plan.poses[pose_id].pose.orientation);
    double yaw3 = transform::tf2::getYaw(transformed_plan.poses[pose_id + 1].pose.orientation);
    if ((std::hypot(oa_x, oa_y) == 0.0 && std::abs(yaw1 - yaw2) > 1e-6) ||
        (std::hypot(ab_x, ab_y) == 0.0 && std::abs(yaw2 - yaw3) > 1e-6)) {
      // returning the distance since the points overlap
      // but are not simply duplicate points (e.g. in place rotation)
      return std::hypot(transformed_plan.poses[pose_id].pose.position.x,
                        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

}  // namespace pure_pursuit_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::pure_pursuit_controller::RegulatedPurePursuitController,
                            autonomy::control::common::ControllerInterface)