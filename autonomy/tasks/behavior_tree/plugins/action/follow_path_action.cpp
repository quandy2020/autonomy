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

#include "autonomy/tasks/behavior_tree/plugins/action/follow_path_action.hpp"

#include <set>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

FollowPathAction::FollowPathAction(const std::string& xml_tag_name, const std::string& action_name,
                                   const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void FollowPathAction::on_tick() {
  commsgs::planning_msgs::Path path;
  if (getInput("path", path)) {
    auto* proto_path = goal_.mutable_path();
    *proto_path->mutable_header() = commsgs::std_msgs::ToProto(path.header);
    proto_path->clear_poses();
    for (const auto& pose : path.poses) {
      auto* proto_pose = proto_path->add_poses();
      *proto_pose = commsgs::geometry_msgs::ToProto(pose);
    }
  }

  std::string controller_id;
  if (getInput("controller_id", controller_id)) {
    goal_.set_controller_id(controller_id);
  }

  std::string goal_checker_id;
  if (getInput("goal_checker_id", goal_checker_id)) {
    goal_.set_goal_checker_id(goal_checker_id);
  }

  std::string progress_checker_id;
  if (getInput("progress_checker_id", progress_checker_id)) {
    goal_.set_progress_checker_id(progress_checker_id);
  }
}

BT::NodeStatus FollowPathAction::on_success() {
  setOutput("error_code_id", static_cast<int32_t>(proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE));
  setOutput("error_msg", std::string(""));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPathAction::on_aborted() {
  if (result_.result) {
    setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
    setOutput("error_msg", result_.result->error_msg());
  } else {
    setOutput("error_code_id", static_cast<int32_t>(proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN));
    setOutput("error_msg", std::string("Action aborted"));
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPathAction::on_cancelled() {
  // Set empty error code, action was cancelled
  setOutput("error_code_id", static_cast<int32_t>(proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE));
  setOutput("error_msg", std::string(""));
  return BT::NodeStatus::SUCCESS;
}

void FollowPathAction::on_timeout() {
  setOutput("error_code_id", static_cast<int32_t>(proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_CONTROLLER_TIMED_OUT));
  setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

void FollowPathAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> /*feedback*/) {
  // Grab the new path
  commsgs::planning_msgs::Path new_path;
  if (getInput("path", new_path)) {
    auto* proto_path = goal_.mutable_path();
    *proto_path->mutable_header() = commsgs::std_msgs::ToProto(new_path.header);
    proto_path->clear_poses();
    for (const auto& pose : new_path.poses) {
      auto* proto_pose = proto_path->add_poses();
      *proto_pose = commsgs::geometry_msgs::ToProto(pose);
    }
    goal_updated_ = true;
  }

  std::string new_controller_id;
  if (getInput("controller_id", new_controller_id) && goal_.controller_id() != new_controller_id) {
    goal_.set_controller_id(new_controller_id);
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  if (getInput("goal_checker_id", new_goal_checker_id) && goal_.goal_checker_id() != new_goal_checker_id) {
    goal_.set_goal_checker_id(new_goal_checker_id);
    goal_updated_ = true;
  }

  std::string new_progress_checker_id;
  if (getInput("progress_checker_id", new_progress_checker_id) &&
      goal_.progress_checker_id() != new_progress_checker_id) {
    goal_.set_progress_checker_id(new_progress_checker_id);
    goal_updated_ = true;
  }
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::FollowPathAction>(name, "follow_path",
                                                                                               config);
  };

  factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::FollowPathAction>("FollowPath", builder);
}
