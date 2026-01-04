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

#include "autonomy/control/checker/position_goal_checker.hpp"

namespace autonomy {
namespace control {
namespace checker {

PositionGoalChecker::PositionGoalChecker()
    : xy_goal_tolerance_(0.25),
      xy_goal_tolerance_sq_(0.0625),
      stateful_(true),
      position_reached_(false) {}

void PositionGoalChecker::Initialize(
    const std::string& plugin_name,
    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    plugin_name_ = plugin_name;

    // TODO: Load parameters from configuration
    // Default values
    xy_goal_tolerance_ = 0.25;
    stateful_ = true;

    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
}

void PositionGoalChecker::Reset() {
    position_reached_ = false;
}

bool PositionGoalChecker::IsGoalReached(
    const commsgs::geometry_msgs::Pose& query_pose,
    const commsgs::geometry_msgs::Pose& goal_pose,
    const commsgs::geometry_msgs::Twist& velocity) {
    // If stateful and position was already reached, maintain state
    if (stateful_ && position_reached_) {
        return true;
    }

    // Check if position is within tolerance
    double dx = query_pose.position.x - goal_pose.position.x;
    double dy = query_pose.position.y - goal_pose.position.y;

    bool position_reached = (dx * dx + dy * dy <= xy_goal_tolerance_sq_);

    // If stateful, remember that we reached the position
    if (stateful_ && position_reached) {
        position_reached_ = true;
    }

    return position_reached;
}

bool PositionGoalChecker::GetTolerances(
    commsgs::geometry_msgs::Pose& pose_tolerance,
    commsgs::geometry_msgs::Twist& vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;

    // Return zero orientation tolerance as we don't check it
    pose_tolerance.orientation.x = 0.0;
    pose_tolerance.orientation.y = 0.0;
    pose_tolerance.orientation.z = 0.0;
    pose_tolerance.orientation.w = 1.0;

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
}

void PositionGoalChecker::SetXYGoalTolerance(double tolerance) {
    xy_goal_tolerance_ = tolerance;
    xy_goal_tolerance_sq_ = tolerance * tolerance;
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy
