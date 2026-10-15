/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/neupan_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

double ExtractYawFromPose(const automsgs::msgs::geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation();
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

void NeupanController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    options_ = options;
    name_ = name;
    plugin_name_ = std::move(name);
    tf_buffer_ = std::move(tf_buffer);
    costmap_wrapper_ = std::move(costmap_wrapper);
    options_neupan_ = options.neupan_controller_options();
    goal_position_tolerance_ = options_neupan_.goal_position_tolerance() > 0.0
                                   ? options_neupan_.goal_position_tolerance()
                                   : 0.25;
    max_encoder_point_count_ =
        options_neupan_.max_obstacle_points_for_encoder() > 0
            ? options_neupan_.max_obstacle_points_for_encoder()
            : 100;
    CacheLaserExtrinsic();

    std::string error;
    if (!EnsureInitialized(&error)) {
        AERROR << "[" << plugin_name_
               << "] NeupanController load failed: " << error;
    } else {
        AINFO << "[" << plugin_name_ << "] encoder loaded from "
              << options_neupan_.neural_encoder_model_path();
    }
}

void NeupanController::CacheLaserExtrinsic() {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (options_neupan_.laser_scan_pose_in_base_size() >= 1) {
        x = options_neupan_.laser_scan_pose_in_base(0);
    }
    if (options_neupan_.laser_scan_pose_in_base_size() >= 2) {
        y = options_neupan_.laser_scan_pose_in_base(1);
    }
    if (options_neupan_.laser_scan_pose_in_base_size() >= 3) {
        yaw = options_neupan_.laser_scan_pose_in_base(2);
    }
    laser_extrinsic_.x = x;
    laser_extrinsic_.y = y;
    laser_extrinsic_.cos_yaw = std::cos(yaw);
    laser_extrinsic_.sin_yaw = std::sin(yaw);
    laser_extrinsic_.range_min = options_neupan_.laser_range_min();
    laser_extrinsic_.range_max = options_neupan_.laser_range_max();
}

RobotFootprint NeupanController::CreateRobotFootprint(
    std::string* error) const {
    std::string kinematics = options_neupan_.robot_kinematics();
    if (kinematics.empty() || kinematics == "diff") {
        kinematics = "differential";
    }
    if (kinematics == "acker") {
        kinematics = "ackermann";
    }

    if (options_neupan_.footprint_vertices_size() >= 6 &&
        options_neupan_.footprint_vertices_size() % 2 == 0) {
        automsgs::msgs::geometry_msgs::Polygon polygon;
        for (int i = 0; i + 1 < options_neupan_.footprint_vertices_size();
             i += 2) {
            auto* point = polygon.add_points();
            point->set_x(
                static_cast<float>(options_neupan_.footprint_vertices(i)));
            point->set_y(
                static_cast<float>(options_neupan_.footprint_vertices(i + 1)));
            point->set_z(0.0f);
        }
        auto footprint = RobotFootprint::CreateFromPolygon(
            kinematics, polygon, options_neupan_.wheelbase());
        if (footprint.GetEdgeCount() < 3) {
            if (error) {
                *error = "invalid footprint_vertices";
            }
            return {};
        }
        return footprint;
    }

    const double length = options_neupan_.footprint_length() > 0.0
                              ? options_neupan_.footprint_length()
                              : 0.8;
    const double width = options_neupan_.footprint_width() > 0.0
                             ? options_neupan_.footprint_width()
                             : 0.6;
    return RobotFootprint::CreateRectangle(kinematics, length, width,
                                           options_neupan_.wheelbase());
}

bool NeupanController::EnsureInitialized(std::string* error) {
    if (planner_) {
        return true;
    }
    const RobotFootprint footprint = CreateRobotFootprint(error);
    if (footprint.GetEdgeCount() <= 0) {
        if (error && error->empty()) {
            *error = "failed to create robot footprint";
        }
        return false;
    }
    auto encoder = DistanceEncoder::Create(
        options_neupan_.neural_encoder_model_path(), footprint,
        options_neupan_.inference_backend(),
        options_neupan_.inference_execution_provider(), error);
    if (!encoder) {
        return false;
    }
    planner_ =
        std::make_unique<MotionPlanner>(options_neupan_, std::move(encoder));
    return true;
}

void NeupanController::Cleanup() {
    planner_.reset();
}

void NeupanController::Activate() {}

void NeupanController::Deactivate() {}

void NeupanController::Reset() {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_cache_.reset();
    has_pose_ = false;
}

void NeupanController::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    auto cache = std::make_shared<PathCache>();
    cache->BuildFrom(plan);
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_cache_ = std::move(cache);
}

void NeupanController::SetSpeedLimit(const double& speed_limit,
                                     const bool& percentage) {
    if (percentage) {
        speed_limit_scale_ = std::max(0.0, std::min(1.0, speed_limit));
    } else if (options_neupan_.max_linear_speed() > 0.0 && speed_limit > 0.0) {
        speed_limit_scale_ = std::max(
            0.0,
            std::min(1.0, speed_limit / options_neupan_.max_linear_speed()));
    }
}

void NeupanController::SetLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan) {
    auto scan = std::make_shared<const automsgs::msgs::sensor_msgs::LaserScan>(
        laser_scan);
    std::lock_guard<std::mutex> lock(laser_scan_mutex_);
    laser_scan_ = std::move(scan);
}

bool NeupanController::IsGoalReached(double distance_tolerance,
                                     double angle_tolerance) {
    (void)angle_tolerance;
    std::shared_ptr<const PathCache> path;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path = path_cache_;
    }
    if (!has_pose_ || !path || path->Empty()) {
        return false;
    }
    const double tolerance = distance_tolerance > 0.0
                                 ? distance_tolerance
                                 : goal_position_tolerance_;
    return path->DistanceToGoal(last_pose_) <= tolerance;
}

uint32 NeupanController::ComputeVelocityCommands(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
    (void)goal_checker;
    cmd_vel.Clear();
    cmd_vel.mutable_header()->CopyFrom(pose.header());

    std::string error;
    if (!EnsureInitialized(&error)) {
        message = error;
        return proto::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    last_pose_.set_x(pose.pose().position().x());
    last_pose_.set_y(pose.pose().position().y());
    last_pose_.set_theta(ExtractYawFromPose(pose.pose()));
    has_pose_ = true;

    std::shared_ptr<const PathCache> path;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path = path_cache_;
    }
    if (!path || path->Empty()) {
        message = "empty path";
        return proto::CONTROLLER_RESULT_INVALID_PATH;
    }

    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> laser_scan;
    {
        std::lock_guard<std::mutex> lock(laser_scan_mutex_);
        laser_scan = laser_scan_;
    }

    if (laser_scan) {
        ExtractObstaclePointsFromLaserScan(*laser_scan, laser_extrinsic_,
                                           max_encoder_point_count_,
                                           &obstacle_points_buffer_);
    } else {
        obstacle_points_buffer_.resize(2, 0);
    }

    automsgs::msgs::geometry_msgs::Twist command;
    const auto result = planner_->ComputeVelocityCommand(
        last_pose_, velocity.twist(), *path, obstacle_points_buffer_, &command,
        nullptr, &message);

    switch (result) {
        case MotionPlanner::Result::kGoalReached:
            cmd_vel.mutable_twist()->mutable_linear()->set_x(0.0);
            cmd_vel.mutable_twist()->mutable_angular()->set_z(0.0);
            return proto::CONTROLLER_RESULT_SUCCESS;
        case MotionPlanner::Result::kCollision:
            cmd_vel.mutable_twist()->mutable_linear()->set_x(0.0);
            cmd_vel.mutable_twist()->mutable_angular()->set_z(0.0);
            return proto::CONTROLLER_RESULT_COLLISION;
        case MotionPlanner::Result::kFailure:
            return proto::CONTROLLER_RESULT_NO_VALID_CMD;
        case MotionPlanner::Result::kSuccess:
            break;
    }

    cmd_vel.mutable_twist()->mutable_linear()->set_x(command.linear().x() *
                                                     speed_limit_scale_);
    cmd_vel.mutable_twist()->mutable_angular()->set_z(command.angular().z());
    return proto::CONTROLLER_RESULT_SUCCESS;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::neupan_controller::NeupanController,
    autonomy::control::common::ControllerInterface)
