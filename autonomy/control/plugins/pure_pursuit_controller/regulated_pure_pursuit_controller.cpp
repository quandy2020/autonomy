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

#include "autonomy/control/plugins/pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "autonomy/common/port.hpp"
#include "autonomy/common/math/angle.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/map/costmap_2d/costmap_filters/filter_values.hpp"

namespace autonomy {
namespace control {
namespace plugins {

void RegulatedPurePursuitController::Configure(
  std::string name, std::shared_ptr<TfBuffer> tf,
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper)
{
    // auto node = parent.lock();
    // node_ = parent;
    // if (!node) {
    //     throw common::ControllerException("Unable to lock node!");
    // }

    // costmap_ros_ = costmap_ros;
    // costmap_ = costmap_ros_->getCostmap();
    // tf_ = tf;
    // plugin_name_ = name;
    // logger_ = node->get_logger();

    // // Handles storage and dynamic configuration of parameters.
    // // Returns pointer to data current param settings.
    // param_handler_ = std::make_unique<ParameterHandler>(
    //     node, plugin_name_, logger_, costmap_->getSizeInMetersX());
    // params_ = param_handler_->getParams();

    // // Handles global path transformations
    // path_handler_ = std::make_unique<PathHandler>(
    //     tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

    // // Checks for imminent collisions
    // collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

    // double control_frequency = 20.0;
    // goal_dist_tol_ = 0.25;  // reasonable default before first update

    // node->get_parameter("controller_frequency", control_frequency);
    // control_duration_ = 1.0 / control_frequency;

    // global_path_pub_ = node->create_publisher<commsgs::planning_msgs::Path>("received_global_plan", 1);
    // carrot_pub_ = node->create_publisher<commsgs::geometry_msgs::PointStamped>("lookahead_point", 1);
    // curvature_carrot_pub_ = node->create_publisher<commsgs::geometry_msgs::PointStamped>(
    //     "curvature_lookahead_point", 1);
    // is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    //     "is_rotating_to_heading", 1);
}

void RegulatedPurePursuitController::Cleanup()
{
//     RCLCPP_INFO(
//         logger_,
//         "Cleaning up controller: %s of type"
//         " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
//         plugin_name_.c_str());
//     global_path_pub_.reset();
//     carrot_pub_.reset();
//     curvature_carrot_pub_.reset();
//     is_rotating_to_heading_pub_.reset();
}

void RegulatedPurePursuitController::Activate()
{
    // RCLCPP_INFO(
    //     logger_,
    //     "Activating controller: %s of type "
    //     "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    //     plugin_name_.c_str());
    // global_path_pub_->on_activate();
    // carrot_pub_->on_activate();
    // curvature_carrot_pub_->on_activate();
    // is_rotating_to_heading_pub_->on_activate();
}

void RegulatedPurePursuitController::Deactivate()
{
    // RCLCPP_INFO(
    //     logger_,
    //     "Deactivating controller: %s of type "
    //     "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    //     plugin_name_.c_str());
    // global_path_pub_->on_deactivate();
    // carrot_pub_->on_deactivate();
    // curvature_carrot_pub_->on_deactivate();
    // is_rotating_to_heading_pub_->on_deactivate();
}

std::unique_ptr<commsgs::geometry_msgs::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const commsgs::geometry_msgs::PoseStamped& carrot_pose)
{
    auto carrot_msg = std::make_unique<commsgs::geometry_msgs::PointStamped>();
    carrot_msg->header = carrot_pose.header;
    carrot_msg->point.x = carrot_pose.pose.position.x;
    carrot_msg->point.y = carrot_pose.pose.position.y;
    carrot_msg->point.z = 0.01;  // publish right over map to stand out
    return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(const commsgs::geometry_msgs::Twist& speed)
{
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = params_->lookahead_dist;
    if (params_->use_velocity_scaled_lookahead_dist) {
        lookahead_dist = std::fabs(speed.linear.x) * params_->lookahead_time;
        lookahead_dist = std::clamp(
        lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
    }

    return lookahead_dist;
}

double calculateCurvature(commsgs::geometry_msgs::Point lookahead_point)
{
    // Find distance^2 to look ahead point (carrot) in robot base frame
    // This is the chord length of the circle
    const double carrot_dist2 =
        (lookahead_point.x * lookahead_point.x) +
        (lookahead_point.y * lookahead_point.y);

    // Find curvature of circle (k = 1 / R)
    if (carrot_dist2 > 0.001) {
        return 2.0 * lookahead_point.y / carrot_dist2;
    } else {
        return 0.0;
    }
}

uint32 RegulatedPurePursuitController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel, 
    common::GoalChecker* goal_checker,
    std::string& message) 
{
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // Update for the current goal checker's state
    commsgs::geometry_msgs::Pose pose_tolerance;
    commsgs::geometry_msgs::Twist vel_tolerance;
    if (!goal_checker->GetTolerances(pose_tolerance, vel_tolerance)) {
        LOG(WARNING) << "Unable to retrieve goal checker's tolerances!";
    } else {
        goal_dist_tol_ = pose_tolerance.position.x;
    }

    // Transform path to robot base frame
    auto transformed_plan = path_handler_->transformGlobalPlan(
        pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
    // global_path_pub_->publish(transformed_plan);

    // Find look ahead distance and point on path and publish
    double lookahead_dist = getLookAheadDistance(velocity.twist);

    // Check for reverse driving
    if (params_->allow_reversing) {
        // Cusp check
        const double dist_to_cusp = findVelocitySignChange(transformed_plan);

        // if the lookahead distance is further than the cusp, use the cusp distance instead
        if (dist_to_cusp < lookahead_dist) {
            lookahead_dist = dist_to_cusp;
        }
    }

    // Get the particular point on the path at the lookahead distance
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    auto rotate_to_path_carrot_pose = carrot_pose;
    // carrot_pub_->publish(createCarrotMsg(carrot_pose));

    double linear_vel, angular_vel;
    double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

    double regulation_curvature = lookahead_curvature;
    if (params_->use_fixed_curvature_lookahead) {
        auto curvature_lookahead_pose = getLookAheadPoint(
        params_->curvature_lookahead_dist,
        transformed_plan, params_->interpolate_curvature_after_goal);
        rotate_to_path_carrot_pose = curvature_lookahead_pose;
        regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
        // curvature_carrot_pub_->publish(createCarrotMsg(curvature_lookahead_pose));
    }

    // Setting the velocity direction
    double x_vel_sign = 1.0;
    if (params_->allow_reversing) {
        x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    }

    linear_vel = params_->desired_linear_vel;

    // Make sure we're in compliance with basic constraints
    // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
    // and rotate_to_path_carrot_pose for the direction carrot pose:
    //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
    //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
    double angle_to_heading;
    if (shouldRotateToGoalHeading(carrot_pose)) {
        is_rotating_to_heading_ = true;
        double angle_to_goal = transform::tf2::getYaw(transform::tf2::Quaternion {
            transformed_plan.poses.back().pose.orientation.x,
            transformed_plan.poses.back().pose.orientation.y,
            transformed_plan.poses.back().pose.orientation.z,
            transformed_plan.poses.back().pose.orientation.w
        });
        rotateToHeading(linear_vel, angular_vel, angle_to_goal, velocity.twist);
    } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
        is_rotating_to_heading_ = true;
        rotateToHeading(linear_vel, angular_vel, angle_to_heading, velocity.twist);
    } else {
        is_rotating_to_heading_ = false;
        applyConstraints(
            regulation_curvature, velocity.twist,
            collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
            linear_vel, x_vel_sign);

        if (cancelling_) {
            const double & dt = control_duration_;
            linear_vel = velocity.twist.linear.x - x_vel_sign * dt * params_->cancel_deceleration;

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
    const double & carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    if (params_->use_collision_detection && 
        collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
        throw common::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    }

    // // Publish whether we are rotating to goal heading
    // std_msgs::msg::Bool is_rotating_to_heading_msg;
    // is_rotating_to_heading_msg.data = is_rotating_to_heading_;
    // is_rotating_to_heading_pub_->publish(is_rotating_to_heading_msg);

    // populate and return message
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;
    return 0;
}

bool RegulatedPurePursuitController::Cancel()
{
    // if false then publish zero velocity
    if (!params_->use_cancel_deceleration) {
        return true;
    }
    cancelling_ = true;
    return finished_cancelling_;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const commsgs::geometry_msgs::PoseStamped& carrot_pose, double& angle_to_path,
  double& x_vel_sign)
{
    // Whether we should rotate robot to rough path heading
    angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    // In case we are reversing
    if (x_vel_sign < 0.0) {
        angle_to_path = autonomy::common::math::NormalizeAngleDifference(angle_to_path + M_PI);
    }
    return params_->use_rotate_to_heading &&
        std::fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const commsgs::geometry_msgs::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const commsgs::geometry_msgs::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

commsgs::geometry_msgs::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const commsgs::geometry_msgs::Point& p1,
  const commsgs::geometry_msgs::Point& p2,
  double r)
{
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
  const double& lookahead_dist,
  const commsgs::planning_msgs::Path& transformed_plan,
  bool interpolate_after_goal)
{
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
            return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
        if (interpolate_after_goal) {
        auto last_pose_it = std::prev(transformed_plan.poses.end());
        auto prev_last_pose_it = std::prev(last_pose_it);

        double end_path_orientation = atan2(
            last_pose_it->pose.position.y - prev_last_pose_it->pose.position.y,
            last_pose_it->pose.position.x - prev_last_pose_it->pose.position.x);

        // Project the last segment out to guarantee it is beyond the look ahead
        // distance
        auto projected_position = last_pose_it->pose.position;
        projected_position.x += cos(end_path_orientation) * lookahead_dist;
        projected_position.y += sin(end_path_orientation) * lookahead_dist;

        // Use the circle intersection to find the position at the correct look
        // ahead distance
        const auto interpolated_position = circleSegmentIntersection(
            last_pose_it->pose.position, projected_position, lookahead_dist);

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
        auto point = circleSegmentIntersection(
        prev_pose_it->pose.position,
        goal_pose_it->pose.position, lookahead_dist);
        commsgs::geometry_msgs::PoseStamped pose;
        pose.header.frame_id = prev_pose_it->header.frame_id;
        pose.header.stamp = goal_pose_it->header.stamp;
        pose.pose.position = point;
        return pose;
    }

    return *goal_pose_it;
}

void RegulatedPurePursuitController::applyConstraints(
  const double& curvature, const commsgs::geometry_msgs::Twist& /*curr_speed*/,
  const double& pose_cost, const commsgs::planning_msgs::Path& path, double& linear_vel, double& sign)
{
    double curvature_vel = linear_vel, cost_vel = linear_vel;

    // limit the linear velocity by curvature
    if (params_->use_regulated_linear_velocity_scaling) {
        curvature_vel = curvatureConstraint(
        linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
    }

    // limit the linear velocity by proximity to obstacles
    if (params_->use_cost_regulated_linear_velocity_scaling) {
        cost_vel = costConstraint(linear_vel, pose_cost, costmap_wrapper_, params_);
    }

    // Use the lowest of the 2 constraints, but above the minimum translational speed
    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

    // Apply constraint to reduce speed on approach to the final goal pose
    linear_vel = approachVelocityConstraint(
        linear_vel, path, params_->min_approach_linear_velocity,
        params_->approach_velocity_scaling_dist);

    // Limit linear velocities to be valid
    linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
    linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::SetPlan(const commsgs::planning_msgs::Path& path)
{
    path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::SetSpeedLimit(const double& speed_limit, const bool& percentage)
{
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
        // Restore default value
        params_->desired_linear_vel = params_->base_desired_linear_vel;
    } else {
        if (percentage) {
            // Speed limit is expressed in % from maximum speed of robot
            params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
        } else {
            // Speed limit is expressed in absolute value
            params_->desired_linear_vel = speed_limit;
        }
    }
}

void RegulatedPurePursuitController::Reset()
{
    cancelling_ = false;
    finished_cancelling_ = false;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const commsgs::planning_msgs::Path& transformed_plan)
{
    auto quat_equal = [](const commsgs::geometry_msgs::Quaternion& q1, 
                         const commsgs::geometry_msgs::Quaternion& q2, 
                         double tol = 1e-6) -> bool {
        return (std::abs(q1.x - q2.x) < tol &&
                std::abs(q1.y - q2.y) < tol &&
                std::abs(q1.z - q2.z) < tol &&
                std::abs(q1.w - q2.w) < tol);
   };

    // Iterating through the transformed global path to determine the position of the cusp
    for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
        // We have two vectors for the dot product OA and AB. Determining the vectors.
        double oa_x = transformed_plan.poses[pose_id].pose.position.x -
        transformed_plan.poses[pose_id - 1].pose.position.x;
        double oa_y = transformed_plan.poses[pose_id].pose.position.y -
        transformed_plan.poses[pose_id - 1].pose.position.y;
        double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
        transformed_plan.poses[pose_id].pose.position.x;
        double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
        transformed_plan.poses[pose_id].pose.position.y;

        /* Checking for the existance of cusp, in the path, using the dot product
        and determine it's distance from the robot. If there is no cusp in the path,
        then just determine the distance to the goal location. */
        const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
        if (dot_prod < 0.0) {
            // returning the distance if there is a cusp
            // The transformed path is in the robots frame, so robot is at the origin
            return std::hypot(
                transformed_plan.poses[pose_id].pose.position.x,
                transformed_plan.poses[pose_id].pose.position.y);
        }

        if ((std::hypot(oa_x, oa_y) == 0.0 &&
            !quat_equal(transformed_plan.poses[pose_id - 1].pose.orientation, 
                transformed_plan.poses[pose_id].pose.orientation)) || 
            (std::hypot(ab_x, ab_y) == 0.0 &&
            !quat_equal(transformed_plan.poses[pose_id].pose.orientation,
                transformed_plan.poses[pose_id + 1].pose.orientation))) {
            // returning the distance since the points overlap
            // but are not simply duplicate points (e.g. in place rotation)
            return std::hypot(
                transformed_plan.poses[pose_id].pose.position.x,
                transformed_plan.poses[pose_id].pose.position.y);
        }
    }

    return std::numeric_limits<double>::max();
}

}  // namespace plugins
}  // namespace control
}  // namespace autonomy