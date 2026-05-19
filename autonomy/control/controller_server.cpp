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

#include "autonomy/control/controller_server.hpp"

#include "autonomy/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/utils/conversions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {

using Time = commsgs::builtin_interfaces::Time;
using namespace autonomy::control::utils;

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options} {
    default_progress_checker_ids_ = {"progress_checker"};
    default_progress_checker_types_ = {
        "nav2_controller::SimpleProgressChecker"};
    default_goal_checker_ids_ = {"goal_checker"};
    default_goal_checker_types_ = {"nav2_controller::SimpleGoalChecker"};
    default_ids_ = {"FollowPath"};
    default_types_ = {"autonomy::control::DWBLocalPlanner"};
    costmap_update_timeout_ =
        commsgs::builtin_interfaces::Duration::FromSeconds(300.0);

    AINFO << "Control server init successfully.";
}

ControllerServer::~ControllerServer() {
    Shutdown();
    AINFO << "Control server shutdown successfully.";
}

void ControllerServer::Start() {
    AINFO << "Starting controller server...";
}

void ControllerServer::Shutdown() {
    AINFO << "Shutting down controller server...";
}

bool ControllerServer::FindControllerId(const std::string& c_name,
                                        std::string& current_controller) {

    return true;
}

bool ControllerServer::FindGoalCheckerId(const std::string& c_name,
                                         std::string& current_goal_checker) {
    return true;
}

bool ControllerServer::FindProgressCheckerId(
    const std::string& c_name, std::string& current_progress_checker) {

    return true;
}

void ControllerServer::ComputeControl() {
    AINFO << "Received a goal, begin computing control effort.";
}

void ControllerServer::SetPlannerPath(
    const commsgs::planning_msgs::Path& path) {
}

void ControllerServer::ComputeAndPublishVelocity() {
}

void ControllerServer::UpdateGlobalPath() {
}

void ControllerServer::PublishVelocity(
    const commsgs::geometry_msgs::TwistStamped& velocity) {
    // 验证速度消息（检查 NaN/Inf）
    if (std::isnan(velocity.twist.linear.x) ||
        std::isinf(velocity.twist.linear.x) ||
        std::isnan(velocity.twist.linear.y) ||
        std::isinf(velocity.twist.linear.y) ||
        std::isnan(velocity.twist.linear.z) ||
        std::isinf(velocity.twist.linear.z) ||
        std::isnan(velocity.twist.angular.x) ||
        std::isinf(velocity.twist.angular.x) ||
        std::isnan(velocity.twist.angular.y) ||
        std::isinf(velocity.twist.angular.y) ||
        std::isnan(velocity.twist.angular.z) ||
        std::isinf(velocity.twist.angular.z)) {
        AERROR
            << "Velocity message contains NaNs or Infs! Ignoring as invalid!";
        return;
    }
}

void ControllerServer::PublishZeroVelocity() {
    commsgs::geometry_msgs::TwistStamped velocity;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.header.frame_id = costmap_wrapper_->getBaseFrameID();
    velocity.header.stamp = Time::Now();
}

void ControllerServer::OnGoalExit() {
    if (publish_zero_velocity_) {
        PublishZeroVelocity();
    }

    // Reset the state of the controllers after the task has ended
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Reset();
    }
}

bool ControllerServer::IsGoalReached() {
    commsgs::geometry_msgs::PoseStamped pose;

    return true;
}

bool ControllerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) {
    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!costmap_wrapper_->getRobotPose(current_pose)) {
        return false;
    }
    pose = current_pose;
    return true;
}

void ControllerServer::SpeedLimitCallback(
    const commsgs::planning_msgs::SpeedLimit::SharedPtr msg) {
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->SetSpeedLimit(msg->speed_limit, msg->percentage);
    }
}

}  // namespace control
}  // namespace autonomy
