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

#include "autonomy/control/checker/pose_progress_checker.hpp"

#include <chrono>

#include "autonomy/common/math/angle.hpp"
#include "autonomy/transform/tf2/utils.h"
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>

namespace autonomy {
namespace control {
namespace checker {

void PoseProgressChecker::Initialize(const std::string& plugin_name) {
    plugin_name_ = plugin_name;
    SimpleProgressChecker::Initialize(plugin_name);
    required_movement_angle_ = 0.5;
}

bool PoseProgressChecker::Check(
    automsgs::msgs::geometry_msgs::PoseStamped& current_pose) {
    automsgs::msgs::geometry_msgs::Pose2D current_pose2d;
    current_pose2d.set_x(current_pose.pose().position().x());
    current_pose2d.set_y(current_pose.pose().position().y());
    current_pose2d.set_theta(
        transform::tf2::getYaw(current_pose.pose().orientation()));

    if (!baseline_pose_set_ ||
        PoseProgressChecker::IsRobotMovedEnough(current_pose2d)) {
        ResetBaselinePose(current_pose2d);
        return true;
    }

    const auto elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - baseline_time_)
                             .count();
    return elapsed <= time_allowance_sec_;
}

bool PoseProgressChecker::IsRobotMovedEnough(
    const automsgs::msgs::geometry_msgs::Pose2D& pose) {
    return PoseDistance(pose, baseline_pose_) > radius_ ||
           PoseAngleDistance(pose, baseline_pose_) > required_movement_angle_;
}

double PoseProgressChecker::PoseAngleDistance(
    const automsgs::msgs::geometry_msgs::Pose2D& pose1,
    const automsgs::msgs::geometry_msgs::Pose2D& pose2) {
    const double diff = pose1.theta() - pose2.theta();
    return std::abs(autonomy::common::math::NormalizeAngleDifference(diff));
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy
