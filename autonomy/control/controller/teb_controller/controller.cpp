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

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/teb_controller/controller.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

void TEBController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
  name_ = std::move(name);
  tf_buffer_ = std::move(tf_buffer);
  costmap_wrapper_ = std::move(costmap_wrapper);
  options_ = options.teb_controller_options();

  path_handler_.initialize(costmap_wrapper_, tf_buffer_, &options_);
  optimizer_.initialize(name_, costmap_wrapper_, &options_,
                        options.controller_frequency());
  AINFO << "Configured TEB Controller: " << name_;
}

void TEBController::Cleanup() { AINFO << "Cleaned up TEB Controller: " << name_; }

void TEBController::Activate() { AINFO << "Activated TEB Controller: " << name_; }

void TEBController::Deactivate() {
  AINFO << "Deactivated TEB Controller: " << name_;
}

void TEBController::Reset() { optimizer_.reset(); }

uint32 TEBController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
  const auto transformed_plan = path_handler_.transformGlobalPlan(pose);
  const auto goal = path_handler_.getLocalGoal(pose);

  map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
  std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *(costmap->getMutex()));

  commsgs::geometry_msgs::Twist robot_speed;
  robot_speed.linear = velocity.twist.linear;
  robot_speed.angular = velocity.twist.angular;

  last_robot_pose_ = pose;
  last_robot_velocity_ = robot_speed;
  last_goal_pose_ = goal.pose;
  last_goal_checker_ = goal_checker;
  has_control_state_ = true;

  try {
    cmd_vel = optimizer_.evalControl(pose, robot_speed, transformed_plan, goal);
  } catch (const common::NoValidControl& ex) {
    message = ex.what();
    return proto::CONTROLLER_RESULT_NO_VALID_CMD;
  }

  return proto::CONTROLLER_RESULT_SUCCESS;
}

void TEBController::SetPlan(const commsgs::planning_msgs::Path& path) {
  path_handler_.setPath(path);
  optimizer_.reset();
  has_control_state_ = false;
}

void TEBController::SetSpeedLimit(const double& speed_limit,
                                 const bool& percentage) {
  (void)speed_limit;
  (void)percentage;
}

bool TEBController::IsGoalReached(double dist_tolerance, double angle_tolerance) {
  (void)dist_tolerance;
  (void)angle_tolerance;
  if (!has_control_state_ || last_goal_checker_ == nullptr) {
    return false;
  }
  return last_goal_checker_->IsGoalReached(last_robot_pose_.pose,
                                           last_goal_pose_,
                                           last_robot_velocity_);
}

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::teb_controller::TEBController,
    autonomy::control::common::ControllerInterface)
