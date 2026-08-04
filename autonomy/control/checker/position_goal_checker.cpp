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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace checker {

PositionGoalChecker::PositionGoalChecker()
    : xy_goal_tolerance_(0.25),
      xy_goal_tolerance_sq_(0.0625),
      path_length_tolerance_(1.0),
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

bool PositionGoalChecker::IsGoalXYReached(
    const automsgs::msgs::geometry_msgs::Pose& query_pose,
    const automsgs::msgs::geometry_msgs::Pose& goal_pose,
    const automsgs::msgs::geometry_msgs::Twist& velocity,
    const automsgs::msgs::nav_msgs::Path& transformed_global_plan) {
    (void)velocity;
    if (map::costmap_2d::utils::calculate_path_length(transformed_global_plan) >
        path_length_tolerance_) {
        return false;
    }
    if (stateful_ && position_reached_) {
        return true;
    }
    const double dx = query_pose.position().x() - goal_pose.position().x();
    const double dy = query_pose.position().y() - goal_pose.position().y();
    const bool position_reached = dx * dx + dy * dy <= xy_goal_tolerance_sq_;
    if (stateful_ && position_reached) {
        position_reached_ = true;
    }
    return position_reached;
}

bool PositionGoalChecker::IsGoalReached(
    const automsgs::msgs::geometry_msgs::Pose& query_pose,
    const automsgs::msgs::geometry_msgs::Pose& goal_pose,
    const automsgs::msgs::geometry_msgs::Twist& velocity) {
    return IsGoalXYReached(query_pose, goal_pose, velocity,
                           automsgs::msgs::nav_msgs::Path{});
}

bool PositionGoalChecker::GetTolerances(
    automsgs::msgs::geometry_msgs::Pose& pose_tolerance,
    automsgs::msgs::geometry_msgs::Twist& vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.mutable_position()->set_x(xy_goal_tolerance_);
    pose_tolerance.mutable_position()->set_y(xy_goal_tolerance_);
    pose_tolerance.mutable_position()->set_z(invalid_field);

    // Return zero orientation tolerance as we don't check it
    pose_tolerance.mutable_orientation()->set_x(0.0);
    pose_tolerance.mutable_orientation()->set_y(0.0);
    pose_tolerance.mutable_orientation()->set_z(0.0);
    pose_tolerance.mutable_orientation()->set_w(1.0);

    vel_tolerance.mutable_linear()->set_x(invalid_field);
    vel_tolerance.mutable_linear()->set_y(invalid_field);
    vel_tolerance.mutable_linear()->set_z(invalid_field);

    vel_tolerance.mutable_angular()->set_x(invalid_field);
    vel_tolerance.mutable_angular()->set_y(invalid_field);
    vel_tolerance.mutable_angular()->set_z(invalid_field);

    return true;
}

void PositionGoalChecker::SetXYGoalTolerance(double tolerance) {
    xy_goal_tolerance_ = tolerance;
    xy_goal_tolerance_sq_ = tolerance * tolerance;
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy
