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

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/utils/conversions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/transform_datatypes.h"

namespace autonomy {
namespace control {

using Time = commsgs::builtin_interfaces::Time;
using namespace autonomy::control::utils;
using autonomy::transform::tf2::TransformException;

namespace {

bool TransformPoseToFrame(
    const commsgs::geometry_msgs::PoseStamped& input_pose,
    commsgs::geometry_msgs::PoseStamped& transformed_pose,
    const std::shared_ptr<transform::Buffer>& tf_buffer,
    const std::string& target_frame, float transform_timeout) {
    if (!tf_buffer) {
        return false;
    }
    try {
        transformed_pose =
            tf_buffer->transform(input_pose, target_frame, transform_timeout);
        return true;
    } catch (const TransformException& ex) {
        AERROR << "Transform error: " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "Transform error: " << ex.what();
    }
    return false;
}

bool GetRobotPoseInFrame(
    commsgs::geometry_msgs::PoseStamped& pose_in_frame,
    const std::shared_ptr<transform::Buffer>& tf_buffer,
    const std::string& global_frame, const std::string& robot_frame,
    float transform_timeout) {
    commsgs::geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.orientation.w = 1.0;
    robot_pose.header.frame_id = robot_frame;
    robot_pose.header.stamp = Time::Now();
    return TransformPoseToFrame(robot_pose, pose_in_frame, tf_buffer,
                                global_frame, transform_timeout);
}

}  // namespace

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options}, publish_zero_velocity_{false} {
    if (options_.has_costmap_2d_options()) {
        costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
            options_.costmap_2d_options(), "local_costmap");
    }
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

void ControllerServer::SetNavigationContext(
    std::shared_ptr<transform::Buffer> tf_buffer,
    const std::string& global_frame, const std::string& robot_base_frame) {
    tf_buffer_ = std::move(tf_buffer);
    if (!global_frame.empty()) {
        global_frame_ = global_frame;
    }
    if (!robot_base_frame.empty()) {
        robot_base_frame_ = robot_base_frame;
    }
}

void ControllerServer::Start() {
    AINFO << "Starting controller server...";
    if (costmap_wrapper_) {
        costmap_wrapper_->Start();
    }
}

void ControllerServer::Shutdown() {
    EndFollowPath();
    AINFO << "Shutting down controller server...";
}

bool ControllerServer::BeginFollowPath(
    const commsgs::planning_msgs::Path& path,
    const std::string& controller_id, const std::string& goal_checker_id,
    const std::string& progress_checker_id) {
    if (path.poses.empty()) {
        AERROR << "BeginFollowPath: path is empty.";
        return false;
    }

    if (!FindControllerId(controller_id, current_controller_)) {
        AERROR << "BeginFollowPath: invalid controller_id.";
        return false;
    }
    if (!FindGoalCheckerId(goal_checker_id, current_goal_checker_)) {
        AERROR << "BeginFollowPath: invalid goal_checker_id.";
        return false;
    }
    if (!FindProgressCheckerId(progress_checker_id,
                              current_progress_checker_)) {
        AERROR << "BeginFollowPath: invalid progress_checker_id.";
        return false;
    }

    SetPlannerPath(path);
    end_pose_ = path.poses.back();
    if (end_pose_.header.frame_id.empty()) {
        end_pose_.header.frame_id = path.header.frame_id.empty()
                                        ? global_frame_
                                        : path.header.frame_id;
    }
    if (end_pose_.header.stamp.sec == 0 && end_pose_.header.stamp.nanosec == 0) {
        end_pose_.header.stamp = Time::Now();
    }
    follow_path_active_ = true;
    AINFO << "BeginFollowPath with " << path.poses.size() << " poses.";
    return true;
}

ControllerServer::FollowPathTickResult ControllerServer::TickFollowPath(
    std::function<bool()> cancel_checker) {
    if (!follow_path_active_) {
        return FollowPathTickResult::Failed;
    }

    if (cancel_checker && cancel_checker()) {
        EndFollowPath();
        return FollowPathTickResult::Cancelled;
    }

    if (IsGoalReached()) {
        EndFollowPath();
        return FollowPathTickResult::Succeeded;
    }

    UpdateGlobalPath();
    ComputeAndPublishVelocity();
    return FollowPathTickResult::Running;
}

void ControllerServer::EndFollowPath() {
    if (!follow_path_active_) {
        return;
    }
    follow_path_active_ = false;
    OnGoalExit();
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
    velocity.header.frame_id =
        costmap_wrapper_ ? costmap_wrapper_->getBaseFrameID() : robot_base_frame_;
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
    if (!follow_path_active_ || !tf_buffer_) {
        return false;
    }

    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!GetRobotPoseInFrame(current_pose, tf_buffer_, global_frame_,
                             robot_base_frame_, 0.1f)) {
        return false;
    }

    commsgs::geometry_msgs::PoseStamped goal_in_global = end_pose_;
    if (!end_pose_.header.frame_id.empty() &&
        end_pose_.header.frame_id != global_frame_ &&
        !TransformPoseToFrame(end_pose_, goal_in_global, tf_buffer_,
                              global_frame_, 0.1f)) {
        return false;
    }

    const double dx =
        current_pose.pose.position.x - goal_in_global.pose.position.x;
    const double dy =
        current_pose.pose.position.y - goal_in_global.pose.position.y;
    const double dist_sq = dx * dx + dy * dy;
    return dist_sq <= goal_reached_tolerance_ * goal_reached_tolerance_;
}

bool ControllerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) {
    if (costmap_wrapper_ && costmap_wrapper_->getRobotPose(pose)) {
        return true;
    }
    return GetRobotPoseInFrame(pose, tf_buffer_, global_frame_, robot_base_frame_,
                               0.1f);
}

void ControllerServer::SpeedLimitCallback(
    const commsgs::planning_msgs::SpeedLimit::SharedPtr msg) {
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->SetSpeedLimit(msg->speed_limit, msg->percentage);
    }
}

}  // namespace control
}  // namespace autonomy
