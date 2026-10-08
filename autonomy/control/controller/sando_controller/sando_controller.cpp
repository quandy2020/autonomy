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

/**
 * @file sando_controller.cpp
 * @brief Plugin entry: costmap ingest, world-to-body twist, and the ControllerInterface lifecycle.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/sando_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "autolink/common/log.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace {

/**
 * @brief Map the options string onto a twist projection.
 *
 * "quadruped", "humanoid", and "wheeled_leg" (also "wheel_leg") are holonomic.
 * Every other string, including empty and "diff_drive", is differential drive.
 *
 * @param name motion_model field after defaults have been applied.
 * @return The kinematics used to rotate the world velocity into the body twist.
 */
GroundMotionModel ParseMotionModel(const std::string& name) {
  if (name == "quadruped") {
    return GroundMotionModel::kQuadruped;
  }
  if (name == "humanoid") {
    return GroundMotionModel::kHumanoid;
  }
  if (name == "wheeled_leg" || name == "wheel_leg") {
    return GroundMotionModel::kWheeledLeg;
  }
  return GroundMotionModel::kDiffDrive;
}

/**
 * @brief Seconds since the steady clock epoch.
 *
 * The value is only compared with itself: obstacle age, replan stamps, and
 * the computation-time filter all use this clock. It is not a ROS time.
 *
 * @return Monotonic time, seconds.
 */
double ReadClockSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

void SandoController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
  name_ = std::move(name);
  tf_buffer_ = std::move(tf);
  costmap_wrapper_ = std::move(costmap_wrapper);
  options_ = options;
  controller_options_ = options.sando_controller_options();
  planner_.Configure(controller_options_);
  controller_options_ = options.sando_controller_options();
  motion_model_ = ParseMotionModel(controller_options_.motion_model().empty() ? "diff_drive" : controller_options_.motion_model());
  speed_scale_ = 1.0;
  AINFO << "Configured SANDO ground controller: " << name_
        << " model=" << (controller_options_.motion_model().empty() ? "diff_drive" : controller_options_.motion_model());
}

void SandoController::Cleanup() {
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan_.Clear();
  has_plan_ = false;
  active_ = false;
  planner_.Reset();
}

void SandoController::Activate() { active_ = true; }

void SandoController::Deactivate() { active_ = false; }

void SandoController::Reset() {
  planner_.Reset();
  speed_scale_ = 1.0;
  goal_distance_ = 1.0e9;
  goal_yaw_error_ = 0.0;
}

void SandoController::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan_ = plan;
  has_plan_ = plan_.poses_size() > 0;
  if (!has_plan_) {
    return;
  }
  const auto& goal = plan_.poses(plan_.poses_size() - 1).pose();
  planner_.SetTerminalGoal(goal.position().x(), goal.position().y(),
                           transform::tf2::getYaw(goal.orientation()));
  goal_distance_ = 1.0e9;
}

void SandoController::AddDynamicObstacle(double x, double y, double velocity_x, double velocity_y, double radius) {
  planner_.AddDynamicObstacle(x, y, velocity_x, velocity_y, radius);
}

void SandoController::SetSpeedLimit(const double& speed_limit, const bool& percentage) {
  const double maximum_velocity = controller_options_.max_linear_vel() > 1e-3 ? controller_options_.max_linear_vel() : 0.8;
  if (percentage) {
    speed_scale_ = std::max(0.0, std::min(1.0, speed_limit / 100.0));
  } else {
    speed_scale_ = std::max(0.0, std::min(1.0, speed_limit / maximum_velocity));
  }
}

uint32 SandoController::ComputeVelocityCommands(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* /*goal_checker*/, std::string& message) {
  if (!active_) {
    message = "SANDO controller inactive";
    return proto::CONTROLLER_RESULT_NOT_INITIALIZED;
  }
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (!has_plan_) {
      message = "SANDO: empty path";
      return proto::CONTROLLER_RESULT_INVALID_PATH;
    }
  }

  const double yaw = transform::tf2::getYaw(pose.pose().orientation());
  State robot;
  robot.mutable_pose()->mutable_position()->set_x(pose.pose().position().x());
  robot.mutable_pose()->mutable_position()->set_y(pose.pose().position().y());
  SetYaw(&robot, yaw);
  const double body_velocity_x = velocity.twist().linear().x();
  const double body_velocity_y = velocity.twist().linear().y();
  robot.mutable_velocity()->mutable_linear()->set_x(std::cos(yaw) * body_velocity_x - std::sin(yaw) * body_velocity_y);
  robot.mutable_velocity()->mutable_linear()->set_y(std::sin(yaw) * body_velocity_x + std::cos(yaw) * body_velocity_y);

  const double now = ReadClockSeconds();
  if (costmap_wrapper_ && costmap_wrapper_->getCostmap()) {
    auto* costmap = costmap_wrapper_->getCostmap();
    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
    planner_.IngestCostmap(*costmap, now, robot.pose().position().x(), robot.pose().position().y());
  }

  State command;
  if (!planner_.ComputeCommand(robot, now, &command, &message)) {
    if (message.empty()) {
      message = "SANDO: no command";
    }
    return proto::CONTROLLER_RESULT_NO_VALID_CMD;
  }

  const double cosine_yaw = std::cos(yaw);
  const double sine_yaw = std::sin(yaw);
  double velocity_x = speed_scale_ * (cosine_yaw * command.velocity().linear().x() + sine_yaw * command.velocity().linear().y());
  double velocity_y = speed_scale_ * (-sine_yaw * command.velocity().linear().x() + cosine_yaw * command.velocity().linear().y());
  double commanded_yaw_rate = command.velocity().angular().z();
  const double maximum_velocity = (controller_options_.max_linear_vel() > 0.0 ? controller_options_.max_linear_vel() : 0.8) * speed_scale_;
  const double maximum_lateral_velocity = (controller_options_.max_lateral_vel() > 0.0 ? controller_options_.max_lateral_vel() : 0.5) * speed_scale_;
  const double maximum_yaw_rate = controller_options_.max_angular_vel() > 0.0 ? controller_options_.max_angular_vel() : 1.0;
  velocity_x = std::max(-maximum_velocity, std::min(maximum_velocity, velocity_x));
  velocity_y = std::max(-maximum_lateral_velocity, std::min(maximum_lateral_velocity, velocity_y));
  commanded_yaw_rate = std::max(-maximum_yaw_rate, std::min(maximum_yaw_rate, commanded_yaw_rate));
  if (motion_model_ == GroundMotionModel::kDiffDrive) {
    const double heading = std::atan2(velocity_y, std::max(1e-6, std::abs(velocity_x)));
    if (std::abs(heading) > 1.0 && std::abs(velocity_y) > std::abs(velocity_x)) {
      velocity_x = 0.0;
    }
    velocity_y = 0.0;
  }

  cmd_vel.mutable_header()->CopyFrom(pose.header());
  cmd_vel.mutable_twist()->mutable_linear()->set_x(velocity_x);
  cmd_vel.mutable_twist()->mutable_linear()->set_y(velocity_y);
  cmd_vel.mutable_twist()->mutable_linear()->set_z(0.0);
  cmd_vel.mutable_twist()->mutable_angular()->set_x(0.0);
  cmd_vel.mutable_twist()->mutable_angular()->set_y(0.0);
  cmd_vel.mutable_twist()->mutable_angular()->set_z(commanded_yaw_rate);

  goal_distance_ = planner_.GetGoalDistance();
  goal_yaw_error_ = planner_.GetGoalYawError();
  return proto::CONTROLLER_RESULT_SUCCESS;
}

bool SandoController::IsGoalReached(double dist_tolerance, double angle_tolerance) {
  const double dist_tol = dist_tolerance > 0.0 ? dist_tolerance : 0.25;
  const double yaw_tol = angle_tolerance > 0.0 ? angle_tolerance : 0.3;
  return planner_.IsTerminalGoalReached() && goal_distance_ <= dist_tol && std::abs(goal_yaw_error_) <= yaw_tol;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
