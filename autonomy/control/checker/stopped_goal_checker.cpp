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

#include "autonomy/control/checker/stopped_goal_checker.hpp"

namespace autonomy {
namespace control {
namespace checker {

StoppedGoalChecker::StoppedGoalChecker()
    : SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25) {}

void StoppedGoalChecker::Initialize(const std::string& plugin_name,
                                    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
  plugin_name_ = plugin_name;
  SimpleGoalChecker::Initialize(plugin_name, costmap_wrapper);

  // nav2_util::declare_parameter_if_not_declared(
  // plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  // nav2_util::declare_parameter_if_not_declared(
  // plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  // node->get_parameter(plugin_name + ".rot_stopped_velocity",
  // rot_stopped_velocity_); node->get_parameter(plugin_name +
  // ".trans_stopped_velocity", trans_stopped_velocity_);
}

bool StoppedGoalChecker::IsGoalReached(const commsgs::geometry_msgs::Pose& query_pose,
                                       const commsgs::geometry_msgs::Pose& goal_pose,
                                       const commsgs::geometry_msgs::Twist& velocity) {
  bool ret = SimpleGoalChecker::IsGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  return std::fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         std::hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

bool StoppedGoalChecker::GetTolerances(commsgs::geometry_msgs::Pose& pose_tolerance,
                                       commsgs::geometry_msgs::Twist& vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  bool rtn = SimpleGoalChecker::GetTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

}  // namespace checker
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::checker::StoppedGoalChecker, autonomy::control::common::GoalChecker)