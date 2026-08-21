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

#include "autonomy/control/checker/simple_goal_checker.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace checker {

SimpleGoalChecker::SimpleGoalChecker()
    : xy_goal_tolerance_(0.25),
      yaw_goal_tolerance_(0.25),
      xy_goal_tolerance_buffer_(0.0),
      path_length_tolerance_(1.0),
      stateful_(true),
      check_xy_(true),
      xy_goal_tolerance_sq_(0.0625),
      xy_goal_tolerance_reset_sq_(0.0625) {}

void SimpleGoalChecker::Initialize(
    const std::string& plugin_name,
    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    plugin_name_ = plugin_name;

    // TODO: Load parameters from configuration
    // Default values
    xy_goal_tolerance_ = 0.25;
    yaw_goal_tolerance_ = 0.25;
    stateful_ = true;

    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
    xy_goal_tolerance_reset_sq_ =
        (xy_goal_tolerance_ + xy_goal_tolerance_buffer_) *
        (xy_goal_tolerance_ + xy_goal_tolerance_buffer_);
}

void SimpleGoalChecker::Reset() {
    check_xy_ = true;
}

void SimpleGoalChecker::SetTolerances(double xy_tolerance, double yaw_tolerance,
                                      bool stateful,
                                      double path_length_tolerance,
                                      double xy_goal_tolerance_buffer) {
    if (xy_tolerance > 0.0) {
        xy_goal_tolerance_ = xy_tolerance;
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
    }
    if (yaw_tolerance > 0.0) {
        yaw_goal_tolerance_ = yaw_tolerance;
    }
    if (path_length_tolerance > 0.0) {
        path_length_tolerance_ = path_length_tolerance;
    }
    xy_goal_tolerance_buffer_ = xy_goal_tolerance_buffer;
    xy_goal_tolerance_reset_sq_ =
        (xy_goal_tolerance_ + xy_goal_tolerance_buffer_) *
        (xy_goal_tolerance_ + xy_goal_tolerance_buffer_);
    stateful_ = stateful;
}

bool SimpleGoalChecker::IsGoalXYReached(
    const automsgs::msgs::geometry_msgs::Pose& query_pose,
    const automsgs::msgs::geometry_msgs::Pose& goal_pose,
    const automsgs::msgs::geometry_msgs::Twist& velocity,
    const automsgs::msgs::nav_msgs::Path& transformed_global_plan) {
    (void)velocity;
    // Only enforce remaining-path length when the caller provided a plan.
    // An empty path (ControllerServer::IsGoalReached) must not be treated as
    // "remaining length 0" or the robot can latch goal far from the true end.
    if (transformed_global_plan.poses_size() > 0 &&
        map::costmap_2d::utils::calculate_path_length(transformed_global_plan) >
            path_length_tolerance_) {
        return false;
    }
    if (check_xy_) {
        const double dx = query_pose.position().x() - goal_pose.position().x();
        const double dy = query_pose.position().y() - goal_pose.position().y();
        if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
            return false;
        }
        if (stateful_) {
            check_xy_ = false;
        }
    } else if (stateful_ && xy_goal_tolerance_buffer_ > 0.0) {
        const double dx = query_pose.position().x() - goal_pose.position().x();
        const double dy = query_pose.position().y() - goal_pose.position().y();
        if (dx * dx + dy * dy > xy_goal_tolerance_reset_sq_) {
            check_xy_ = true;
            return false;
        }
    }
    return true;
}

bool SimpleGoalChecker::IsGoalReached(
    const automsgs::msgs::geometry_msgs::Pose& query_pose,
    const automsgs::msgs::geometry_msgs::Pose& goal_pose,
    const automsgs::msgs::geometry_msgs::Twist& velocity) {
    (void)velocity;
    // Terminal check always requires XY + yaw against the true goal pose.
    // Do not reuse stateful check_xy_ from mid-path IsGoalXYReached() calls.
    const double dx = query_pose.position().x() - goal_pose.position().x();
    const double dy = query_pose.position().y() - goal_pose.position().y();
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
        return false;
    }
    const double dyaw = autonomy::common::math::AngleDiff(
        transform::tf2::getYaw(query_pose.orientation()),
        transform::tf2::getYaw(goal_pose.orientation()));
    return std::abs(dyaw) <= yaw_goal_tolerance_;
}

bool SimpleGoalChecker::GetTolerances(
    automsgs::msgs::geometry_msgs::Pose& pose_tolerance,
    automsgs::msgs::geometry_msgs::Twist& vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.mutable_position()->set_x(xy_goal_tolerance_);
    pose_tolerance.mutable_position()->set_y(xy_goal_tolerance_);
    pose_tolerance.mutable_position()->set_z(invalid_field);
    *pose_tolerance.mutable_orientation() =
        map::costmap_2d::utils::OrientationAroundZAxis(yaw_goal_tolerance_);

    vel_tolerance.mutable_linear()->set_x(invalid_field);
    vel_tolerance.mutable_linear()->set_y(invalid_field);
    vel_tolerance.mutable_linear()->set_z(invalid_field);

    vel_tolerance.mutable_angular()->set_x(invalid_field);
    vel_tolerance.mutable_angular()->set_y(invalid_field);
    vel_tolerance.mutable_angular()->set_z(invalid_field);

    return true;
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy