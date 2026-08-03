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

#include "autonomy/control/checker/simple_progress_checker.hpp"

#include "autonomy/transform/tf2/utils.h"
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>

namespace autonomy {
namespace control {
namespace checker {

void SimpleProgressChecker::Initialize(const std::string& plugin_name) {
    plugin_name_ = plugin_name;
    radius_ = 0.5;  // Default value, can be configured via parameters
}

bool SimpleProgressChecker::Check(
    automsgs::msgs::geometry_msgs::PoseStamped& current_pose) {
    // Convert Pose to Pose2D
    automsgs::msgs::geometry_msgs::Pose2D current_pose2d;
    current_pose2d.set_x(current_pose.pose().position().x());
    current_pose2d.set_y(current_pose.pose().position().y());
    current_pose2d.set_theta(transform::tf2::getYaw(current_pose.pose().orientation()));

    if ((!baseline_pose_set_) || (IsRobotMovedEnough(current_pose2d))) {
        ResetBaselinePose(current_pose2d);
        return true;
    }
    // If robot hasn't moved enough, progress check fails
    return false;
}

void SimpleProgressChecker::Reset() {
    baseline_pose_set_ = false;
}

void SimpleProgressChecker::SetRequiredMovementRadius(double radius) {
    if (radius > 0.0) {
        radius_ = radius;
    }
}

void SimpleProgressChecker::ResetBaselinePose(
    const automsgs::msgs::geometry_msgs::Pose2D& pose) {
    baseline_pose_ = pose;
    // baseline_time_ = clock_->now();
    baseline_pose_set_ = true;
}

bool SimpleProgressChecker::IsRobotMovedEnough(
    const automsgs::msgs::geometry_msgs::Pose2D& pose) {
    return PoseDistance(pose, baseline_pose_) > radius_;
}

double SimpleProgressChecker::PoseDistance(
    const automsgs::msgs::geometry_msgs::Pose2D& pose1,
    const automsgs::msgs::geometry_msgs::Pose2D& pose2) {
    double dx = pose1.x() - pose2.x();
    double dy = pose1.y() - pose2.y();

    return std::hypot(dx, dy);
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy