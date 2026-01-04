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

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace checker {

SimpleGoalChecker::SimpleGoalChecker()
    : xy_goal_tolerance_(0.25),
      yaw_goal_tolerance_(0.25),
      stateful_(true),
      check_xy_(true),
      xy_goal_tolerance_sq_(0.0625) {}

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
}

void SimpleGoalChecker::Reset() {
    check_xy_ = true;
}

bool SimpleGoalChecker::IsGoalReached(
    const commsgs::geometry_msgs::Pose& query_pose,
    const commsgs::geometry_msgs::Pose& goal_pose,
    const commsgs::geometry_msgs::Twist& velocity) {
    if (check_xy_) {
        double dx = query_pose.position.x - goal_pose.position.x;
        double dy = query_pose.position.y - goal_pose.position.y;
        if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
            return false;
        }
        // We are within the window
        // If we are stateful, change the state.
        if (stateful_) {
            check_xy_ = false;
        }
    }
    double dyaw = autonomy::common::math::AngleDiff(
        transform::tf2::getYaw(query_pose.orientation),
        transform::tf2::getYaw(goal_pose.orientation));
    return std::abs(dyaw) <= yaw_goal_tolerance_;
}

bool SimpleGoalChecker::GetTolerances(
    commsgs::geometry_msgs::Pose& pose_tolerance,
    commsgs::geometry_msgs::Twist& vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation =
        map::costmap_2d::utils::OrientationAroundZAxis(yaw_goal_tolerance_);

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy