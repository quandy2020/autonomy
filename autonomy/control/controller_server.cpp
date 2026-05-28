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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/checker/simple_progress_checker.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller_factory.hpp"
#include "autonomy/transform/tf2/utils.h"
#include "autonomy/control/utils/conversions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/buffer_core.h"
#include "autonomy/transform/tf2/transform_datatypes.h"

namespace autonomy {
namespace control {

using Time = commsgs::builtin_interfaces::Time;
using namespace autonomy::control::utils;
using autonomy::transform::tf2::BufferCore;
using autonomy::transform::tf2::TransformException;

namespace {

constexpr double kMaxCmdVelLinear = 10.0;
constexpr double kMaxCmdVelAngular = 10.0;

commsgs::geometry_msgs::Twist ZeroTwist() {
  commsgs::geometry_msgs::Twist twist{};
  return twist;
}

bool IsValidTwist(const commsgs::geometry_msgs::Twist& twist) {
  auto finite_and_bounded = [](float value, double max_abs) {
    return std::isfinite(static_cast<double>(value)) &&
           std::abs(static_cast<double>(value)) <= max_abs;
  };
  return finite_and_bounded(twist.linear.x, kMaxCmdVelLinear) &&
         finite_and_bounded(twist.linear.y, kMaxCmdVelLinear) &&
         finite_and_bounded(twist.linear.z, kMaxCmdVelLinear) &&
         finite_and_bounded(twist.angular.x, kMaxCmdVelAngular) &&
         finite_and_bounded(twist.angular.y, kMaxCmdVelAngular) &&
         finite_and_bounded(twist.angular.z, kMaxCmdVelAngular);
}

bool TransformPoseToFrame(
    const commsgs::geometry_msgs::PoseStamped& input_pose,
    commsgs::geometry_msgs::PoseStamped& transformed_pose,
    const std::shared_ptr<transform::Buffer>& tf_buffer,
    const std::string& target_frame, float transform_timeout) {
    if (!tf_buffer) {
        return false;
    }
    if (input_pose.header.frame_id.empty() ||
        input_pose.header.frame_id == target_frame) {
        transformed_pose = input_pose;
        transformed_pose.header.frame_id = target_frame;
        return true;
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
  if (!tf_buffer) {
    return false;
  }
  try {
    const Time latest{};
    std::string err;
    if (!tf_buffer->canTransform(global_frame, robot_frame, latest,
                                transform_timeout, &err)) {
      AERROR << "canTransform failed for " << robot_frame << " in "
             << global_frame << ": " << err;
      return false;
    }
    const geometry_msgs::TransformStamped gt =
        static_cast<BufferCore&>(*tf_buffer).lookupTransform(
            global_frame, robot_frame, 0ULL);
    pose_in_frame.header.frame_id = global_frame;
    pose_in_frame.header.stamp = latest;
    pose_in_frame.pose.position.x = gt.transform.translation.x;
    pose_in_frame.pose.position.y = gt.transform.translation.y;
    pose_in_frame.pose.position.z = gt.transform.translation.z;
    pose_in_frame.pose.orientation.x = gt.transform.rotation.x;
    pose_in_frame.pose.orientation.y = gt.transform.rotation.y;
    pose_in_frame.pose.orientation.z = gt.transform.rotation.z;
    pose_in_frame.pose.orientation.w = gt.transform.rotation.w;
    return true;
  } catch (const TransformException& ex) {
    AERROR << "Transform error: " << ex.what();
  } catch (const std::exception& ex) {
    AERROR << "Transform error: " << ex.what();
  }
  return false;
}

template <typename Map>
bool ResolvePluginId(const std::string& requested,
                     const std::vector<std::string>& default_ids,
                     const Map& plugins, std::string& resolved) {
    if (!requested.empty()) {
        if (plugins.find(requested) != plugins.end()) {
            resolved = requested;
            return true;
        }
        return false;
    }
    if (plugins.size() == 1) {
        resolved = plugins.begin()->first;
        return true;
    }
    if (!default_ids.empty() &&
        plugins.find(default_ids.front()) != plugins.end()) {
        resolved = default_ids.front();
        return true;
    }
    return false;
}

}  // namespace

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options} {
  if (options_.has_costmap_2d_options() &&
      options_.costmap_2d_options().enabled()) {
    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        options_.costmap_2d_options(), "local_costmap");
  }

  default_progress_checker_ids_ = {"progress_checker"};
  default_progress_checker_types_ = {
      "nav2_controller::SimpleProgressChecker"};
  default_goal_checker_ids_ = {"goal_checker"};
  default_goal_checker_types_ = {"nav2_controller::SimpleGoalChecker"};
  default_ids_ = {"FollowPath", "graceful_controller", "nmpc_controller",
                  "tdmpc_controller", "mppi_controller"};
  default_types_ = {"graceful_controller", "graceful_controller",
                    "nmpc_controller", "tdmpc_controller", "mppi_controller"};

  controller_frequency_ = options_.controller_frequency() > 0.0
                              ? options_.controller_frequency()
                              : 20.0;
  failure_tolerance_ = options_.failure_tolerance() > 0.0
                           ? options_.failure_tolerance()
                           : 0.0;
  publish_zero_velocity_ = options_.publish_zero_velocity();
  min_x_velocity_threshold_ = 0.0001;
  min_y_velocity_threshold_ = 0.0001;
  min_theta_velocity_threshold_ = 0.0001;

  if (options_.has_checker_options() &&
      options_.checker_options().has_goal_checker()) {
    const auto& gc = options_.checker_options().goal_checker();
    if (gc.xy_goal_tolerance() > 0.0) {
      goal_reached_tolerance_ = gc.xy_goal_tolerance();
    }
  }

  costmap_update_timeout_ =
      commsgs::builtin_interfaces::Duration::FromSeconds(300.0);

  tf_buffer_ = std::shared_ptr<transform::Buffer>(
      transform::Buffer::Instance(), [](transform::Buffer*) {});

  odom_smoother_ = std::make_shared<utils::OdomSmoother>();

  AINFO << "Control server init successfully.";
}

ControllerServer::~ControllerServer() {
  Shutdown();
  AINFO << "Control server shutdown successfully.";
}

void ControllerServer::SetNavigationContext(
    std::shared_ptr<transform::Buffer> tf_buffer,
    const std::string& global_frame, const std::string& robot_base_frame) {
  if (tf_buffer) {
    tf_buffer_ = std::move(tf_buffer);
  }
  if (!global_frame.empty()) {
    global_frame_ = global_frame;
  }
  if (!robot_base_frame.empty()) {
    robot_base_frame_ = robot_base_frame;
  }
}

void ControllerServer::SetSharedCostmap(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
  if (costmap) {
    costmap_wrapper_ = std::move(costmap);
  }
  LoadPlugins();
}

void ControllerServer::UpdateOdometry(
    const commsgs::planning_msgs::Odometry& odom) {
  if (odom_smoother_) {
    odom_smoother_->UpdateOdometry(odom);
  }
}

bool ControllerServer::HasOdometry() const {
  return odom_smoother_ && odom_smoother_->HasOdometry();
}

bool ControllerServer::GetLatestOdometry(
    commsgs::planning_msgs::Odometry& odom) const {
  return odom_smoother_ && odom_smoother_->GetLatestOdometry(odom);
}

void ControllerServer::LoadPlugins() {
  if (plugins_loaded_) {
    return;
  }
  if (!costmap_wrapper_) {
    AWARN << "ControllerServer: no costmap; skipping controller plugins.";
    plugins_loaded_ = true;
    return;
  }
  if (!tf_buffer_) {
    tf_buffer_ = std::shared_ptr<transform::Buffer>(
        transform::Buffer::Instance(), [](transform::Buffer*) {});
  }

  progress_checker_ids_ = default_progress_checker_ids_;
  progress_checker_types_ = default_progress_checker_types_;
  for (size_t i = 0; i < progress_checker_ids_.size(); ++i) {
    auto checker = std::make_shared<checker::SimpleProgressChecker>();
    checker->Initialize(progress_checker_ids_[i]);
    checker->Reset();
    progress_checkers_.insert({progress_checker_ids_[i], checker});
  }

  goal_checker_ids_ = default_goal_checker_ids_;
  goal_checker_types_ = default_goal_checker_types_;
  for (size_t i = 0; i < goal_checker_ids_.size(); ++i) {
    auto goal_checker = std::make_shared<checker::SimpleGoalChecker>();
    goal_checker->Initialize(goal_checker_ids_[i], costmap_wrapper_);
    goal_checker->Reset();
    goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
  }

  controller_ids_ = default_ids_;
  controller_types_ = default_types_;
  for (size_t i = 0; i < controller_ids_.size(); ++i) {
    const std::string type =
        i < controller_types_.size() ? controller_types_[i]
                                     : "graceful_controller";
    const std::string resolved = ControllerFactory::ResolveControllerTypeId(type);
    ControllerCreateContext ctx{options_, controller_ids_[i], tf_buffer_,
                                costmap_wrapper_};
    auto controller = ControllerFactory::Create(type, ctx);
    if (!controller) {
      AWARN << "Failed to create controller plugin: " << controller_ids_[i];
      continue;
    }
    controllers_.insert({controller_ids_[i], controller});
    AINFO << "Created controller plugin: " << controller_ids_[i]
          << " (type=" << type << ", factory_id=" << resolved << ")";
  }

  progress_checker_ids_concat_.clear();
  goal_checker_ids_concat_.clear();
  controller_ids_concat_.clear();
  for (const auto& id : progress_checker_ids_) {
    if (!progress_checker_ids_concat_.empty()) {
      progress_checker_ids_concat_ += " ";
    }
    progress_checker_ids_concat_ += id;
  }
  for (const auto& id : goal_checker_ids_) {
    if (!goal_checker_ids_concat_.empty()) {
      goal_checker_ids_concat_ += " ";
    }
    goal_checker_ids_concat_ += id;
  }
  for (const auto& id : controller_ids_) {
    if (!controller_ids_concat_.empty()) {
      controller_ids_concat_ += " ";
    }
    controller_ids_concat_ += id;
  }

  AINFO << "Controller server plugins: controllers [" << controller_ids_concat_
        << "], goal_checkers [" << goal_checker_ids_concat_
        << "], progress_checkers [" << progress_checker_ids_concat_ << "]";

  plugins_loaded_ = true;
}

void ControllerServer::ActivateControllers() {
  if (controllers_active_) {
    return;
  }
  for (auto& entry : controllers_) {
    entry.second->Activate();
  }
  controllers_active_ = true;
}

void ControllerServer::DeactivateControllers() {
  if (!controllers_active_) {
    return;
  }
  for (auto& entry : controllers_) {
    entry.second->Deactivate();
  }
  controllers_active_ = false;
}

void ControllerServer::Start() {
  AINFO << "Starting controller server...";
  if (costmap_wrapper_ &&
      (!options_.has_costmap_2d_options() ||
       options_.costmap_2d_options().enabled())) {
    costmap_wrapper_->Start();
  }
  ActivateControllers();
}

void ControllerServer::Shutdown() {
  EndFollowPath();
  DeactivateControllers();
  for (auto& entry : controllers_) {
    entry.second->Cleanup();
  }
  controllers_.clear();
  goal_checkers_.clear();
  progress_checkers_.clear();
  plugins_loaded_ = false;
  AINFO << "Shutting down controller server...";
}

common::ControllerInterface* ControllerServer::GetController(
    const std::string& id) {
  auto it = controllers_.find(id);
  return it != controllers_.end() ? it->second.get() : nullptr;
}

common::GoalChecker* ControllerServer::GetGoalChecker(const std::string& id) {
  auto it = goal_checkers_.find(id);
  return it != goal_checkers_.end() ? it->second.get() : nullptr;
}

common::ProgressChecker* ControllerServer::GetProgressChecker(
    const std::string& id) {
  auto it = progress_checkers_.find(id);
  return it != progress_checkers_.end() ? it->second.get() : nullptr;
}

bool ControllerServer::BeginFollowPath(
    const commsgs::planning_msgs::Path& path,
    const std::string& controller_id, const std::string& goal_checker_id,
    const std::string& progress_checker_id) {
  if (path.poses.empty()) {
    AERROR << "BeginFollowPath: path is empty.";
    return false;
  }

  if (!plugins_loaded_) {
    AWARN << "BeginFollowPath: plugins not loaded yet.";
    return false;
  }

  std::string resolved_controller;
  if (!ResolvePluginId(controller_id, default_ids_, controllers_,
                       resolved_controller)) {
    AERROR << "BeginFollowPath: invalid controller_id '" << controller_id
           << "'. Available: " << controller_ids_concat_;
    return false;
  }
  std::string resolved_goal_checker;
  if (!ResolvePluginId(goal_checker_id, default_goal_checker_ids_,
                       goal_checkers_, resolved_goal_checker)) {
    AERROR << "BeginFollowPath: invalid goal_checker_id.";
    return false;
  }
  std::string resolved_progress_checker;
  if (!ResolvePluginId(progress_checker_id, default_progress_checker_ids_,
                       progress_checkers_, resolved_progress_checker)) {
    AERROR << "BeginFollowPath: invalid progress_checker_id.";
    return false;
  }

  // If a follow task is already active with the same plugin selection,
  // update the reference path in-place instead of resetting controller state.
  // This keeps global re-planning and local tracking synchronized.
  if (follow_path_active_ && resolved_controller == current_controller_ &&
      resolved_goal_checker == current_goal_checker_ &&
      resolved_progress_checker == current_progress_checker_) {
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
    return true;
  }

  current_controller_ = resolved_controller;
  current_goal_checker_ = resolved_goal_checker;
  current_progress_checker_ = resolved_progress_checker;

  if (auto* progress = GetProgressChecker(current_progress_checker_)) {
    progress->Reset();
  }
  if (auto* goal = GetGoalChecker(current_goal_checker_)) {
    goal->Reset();
  }
  if (auto* ctrl = GetController(current_controller_)) {
    ctrl->Reset();
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
  last_valid_cmd_time_ = Time::Now();
  AINFO << "BeginFollowPath with " << path.poses.size()
        << " poses using controller '" << current_controller_ << "'.";
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
  try {
    ComputeAndPublishVelocity();
  } catch (const common::InvalidPath& ex) {
    AERROR << ex.what();
    EndFollowPath();
    return FollowPathTickResult::Failed;
  } catch (const common::ControllerTFError& ex) {
    AERROR << ex.what();
    EndFollowPath();
    return FollowPathTickResult::Failed;
  } catch (const common::NoValidControl& ex) {
    AERROR << ex.what();
    EndFollowPath();
    return FollowPathTickResult::Failed;
  }

  return FollowPathTickResult::Running;
}

void ControllerServer::EndFollowPath() {
  if (!follow_path_active_) {
    return;
  }
  follow_path_active_ = false;
  OnGoalExit();
}

bool ControllerServer::BeginRecoveryMotion(
    const RecoveryMotionCommand& cmd) {
  if (recovery_active_) {
    EndRecoveryMotion();
  }
  if (follow_path_active_) {
    EndFollowPath();
  }
  if (!GetRobotPose(recovery_start_pose_)) {
    AERROR << "BeginRecoveryMotion: robot pose unavailable.";
    return false;
  }
  recovery_cmd_ = cmd;
  recovery_start_yaw_ = transform::tf2::getYaw(
      recovery_start_pose_.pose.orientation);
  recovery_deadline_ = Time::Now();
  recovery_deadline_.sec +=
      static_cast<int32_t>(cmd.time_allowance_sec);
  recovery_deadline_.nanosec += static_cast<uint32_t>(
      (cmd.time_allowance_sec -
       static_cast<double>(static_cast<int32_t>(cmd.time_allowance_sec))) *
      1e9);
  recovery_active_ = true;
  return true;
}

ControllerServer::RecoveryTickResult ControllerServer::TickRecoveryMotion(
    std::function<bool()> cancel_checker) {
  if (!recovery_active_) {
    return RecoveryTickResult::Failed;
  }
  if (cancel_checker && cancel_checker()) {
    EndRecoveryMotion();
    return RecoveryTickResult::Cancelled;
  }
  const auto now = Time::Now();
  const int64_t deadline_ns =
      static_cast<int64_t>(recovery_deadline_.sec) * 1000000000LL +
      recovery_deadline_.nanosec;
  const int64_t now_ns =
      static_cast<int64_t>(now.sec) * 1000000000LL + now.nanosec;
  if (now_ns >= deadline_ns) {
    EndRecoveryMotion();
    return RecoveryTickResult::Failed;
  }

  commsgs::geometry_msgs::PoseStamped pose;
  if (!GetRobotPose(pose)) {
    EndRecoveryMotion();
    return RecoveryTickResult::Failed;
  }

  commsgs::geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = robot_base_frame_;

  const double speed = std::abs(recovery_cmd_.speed);
  switch (recovery_cmd_.type) {
    case RecoveryMotionType::Spin: {
      const double current_yaw =
          transform::tf2::getYaw(pose.pose.orientation);
      const double rotated = std::abs(
          ::autonomy::common::NormalizeAngleDifference(current_yaw -
                                                     recovery_start_yaw_));
      if (rotated >= std::abs(recovery_cmd_.distance)) {
        EndRecoveryMotion();
        return RecoveryTickResult::Succeeded;
      }
      const double sign =
          recovery_cmd_.distance >= 0.0 ? 1.0 : -1.0;
      cmd_vel.twist.angular.z = sign * speed;
      break;
    }
    case RecoveryMotionType::BackUp:
    case RecoveryMotionType::DriveOnHeading: {
      const double dx =
          pose.pose.position.x - recovery_start_pose_.pose.position.x;
      const double dy =
          pose.pose.position.y - recovery_start_pose_.pose.position.y;
      const double traveled = std::hypot(dx, dy);
      if (traveled >= std::abs(recovery_cmd_.distance)) {
        EndRecoveryMotion();
        return RecoveryTickResult::Succeeded;
      }
      cmd_vel.twist.linear.x =
          recovery_cmd_.type == RecoveryMotionType::BackUp ? -speed : speed;
      break;
    }
  }

  PublishVelocity(cmd_vel);
  return RecoveryTickResult::Running;
}

void ControllerServer::EndRecoveryMotion() {
  if (!recovery_active_) {
    return;
  }
  recovery_active_ = false;
  PublishZeroVelocity();
}

bool ControllerServer::FindControllerId(const std::string& c_name,
                                        std::string& current_controller) {
  return ResolvePluginId(c_name, default_ids_, controllers_,
                         current_controller);
}

bool ControllerServer::FindGoalCheckerId(const std::string& c_name,
                                         std::string& current_goal_checker) {
  return ResolvePluginId(c_name, default_goal_checker_ids_, goal_checkers_,
                         current_goal_checker);
}

bool ControllerServer::FindProgressCheckerId(
    const std::string& c_name, std::string& current_progress_checker) {
  return ResolvePluginId(c_name, default_progress_checker_ids_,
                         progress_checkers_, current_progress_checker);
}

void ControllerServer::ComputeControl() {
  AINFO << "Received a goal, begin computing control effort.";
  ComputeAndPublishVelocity();
}

void ControllerServer::SetPlannerPath(
    const commsgs::planning_msgs::Path& path) {
  current_path_ = path;
  if (auto* ctrl = GetController(current_controller_)) {
    ctrl->SetPlan(path);
  }
}

void ControllerServer::ComputeAndPublishVelocity() {
  auto* controller = GetController(current_controller_);
  if (!controller) {
    return;
  }

  commsgs::geometry_msgs::PoseStamped pose;
  if (!GetRobotPose(pose)) {
    throw common::ControllerTFError("Failed to obtain robot pose.");
  }

  commsgs::geometry_msgs::TwistStamped velocity;
  velocity.header.stamp = Time::Now();
  velocity.header.frame_id = robot_base_frame_;
  if (odom_smoother_ && odom_smoother_->HasOdometry()) {
    velocity = odom_smoother_->getTwistStamped();
    velocity.header.frame_id = robot_base_frame_;
  }

  commsgs::geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = Time::Now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist = ZeroTwist();

  std::string message;
  auto* goal_checker = GetGoalChecker(current_goal_checker_);
  const uint32_t result = controller->ComputeVelocityCommands(
      pose, velocity, cmd_vel, goal_checker, message);

  if (result != 0) {
    if (!message.empty()) {
      AWARN << "Controller '" << current_controller_
            << "' returned code " << result << ": " << message;
    }
    throw common::NoValidControl("Controller failed to compute velocity.");
  }

  if (!IsValidTwist(cmd_vel.twist)) {
    throw common::NoValidControl(
        "Controller returned invalid (NaN/Inf) velocity.");
  }

  PublishVelocity(cmd_vel);
  last_valid_cmd_time_ = Time::Now();
}

void ControllerServer::UpdateGlobalPath() {
  // Global path is installed once in BeginFollowPath / SetPlannerPath.
  // Do not call ControllerInterface::SetPlan every control tick: that resets
  // internal state (e.g. graceful_controller do_initial_rotation_ /
  // goal_reached_) and prevents path tracking — only in-place rotation at
  // v_angular_min_in_place.
  (void)current_path_;
}

void ControllerServer::PublishVelocity(
    const commsgs::geometry_msgs::TwistStamped& velocity) {
  if (!IsValidTwist(velocity.twist)) {
    AERROR << "Velocity message contains NaNs or Infs! Ignoring as invalid!";
    return;
  }

  last_cmd_vel_ = velocity;
  AINFO << "cmd_vel [" << velocity.header.frame_id << "] linear=("
        << velocity.twist.linear.x << ", " << velocity.twist.linear.y << ", "
        << velocity.twist.linear.z << ") angular=(" << velocity.twist.angular.x
        << ", " << velocity.twist.angular.y << ", "
        << velocity.twist.angular.z << ")";
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
  PublishVelocity(velocity);
}

commsgs::geometry_msgs::TwistStamped ControllerServer::GetLastCmdVel() const {
  return last_cmd_vel_;
}

void ControllerServer::OnGoalExit() {
  if (publish_zero_velocity_) {
    PublishZeroVelocity();
  }

  for (auto& entry : controllers_) {
    entry.second->Reset();
  }
  if (auto* goal = GetGoalChecker(current_goal_checker_)) {
    goal->Reset();
  }
  if (auto* progress = GetProgressChecker(current_progress_checker_)) {
    progress->Reset();
  }
}

bool ControllerServer::IsGoalReached() {
  if (!follow_path_active_) {
    return false;
  }

  commsgs::geometry_msgs::PoseStamped current_pose;
  if (!GetRobotPose(current_pose)) {
    return false;
  }

  commsgs::geometry_msgs::Twist velocity;
  if (odom_smoother_ && odom_smoother_->HasOdometry()) {
    velocity = odom_smoother_->getTwist();
  }

  if (auto* goal_checker = GetGoalChecker(current_goal_checker_)) {
    commsgs::geometry_msgs::PoseStamped goal_in_global = end_pose_;
    if (!end_pose_.header.frame_id.empty() &&
        end_pose_.header.frame_id != global_frame_ &&
        !TransformPoseToFrame(end_pose_, goal_in_global, tf_buffer_,
                              global_frame_, transform_tolerance_)) {
      return false;
    }
    return goal_checker->IsGoalReached(current_pose.pose, goal_in_global.pose,
                                         velocity);
  }

  commsgs::geometry_msgs::PoseStamped goal_in_global = end_pose_;
  if (!end_pose_.header.frame_id.empty() &&
      end_pose_.header.frame_id != global_frame_ &&
      !TransformPoseToFrame(end_pose_, goal_in_global, tf_buffer_,
                            global_frame_, transform_tolerance_)) {
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
  if (odom_smoother_) {
    commsgs::planning_msgs::Odometry odom;
    if (odom_smoother_->GetLatestOdometry(odom) &&
        !odom.header.frame_id.empty()) {
      pose.header.frame_id = global_frame_;
      pose.header.stamp = odom.header.stamp;
      pose.pose = odom.pose.pose;
      return true;
    }
  }
  if (GetRobotPoseInFrame(pose, tf_buffer_, global_frame_, robot_base_frame_,
                          transform_tolerance_)) {
    return true;
  }
  if (costmap_wrapper_ && costmap_wrapper_->getRobotPose(pose)) {
    return true;
  }
  return false;
}

void ControllerServer::SpeedLimitCallback(
    const commsgs::planning_msgs::SpeedLimit::SharedPtr msg) {
  if (!msg) {
    return;
  }
  for (auto& entry : controllers_) {
    entry.second->SetSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

void ControllerServer::ApplySpeedLimit(
    const commsgs::planning_msgs::SpeedLimit& msg) {
  SpeedLimitCallback(
    std::make_shared<commsgs::planning_msgs::SpeedLimit>(msg));
}

}  // namespace control
}  // namespace autonomy
