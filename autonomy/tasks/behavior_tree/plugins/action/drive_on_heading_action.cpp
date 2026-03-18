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

#include "autonomy/tasks/behavior_tree/plugins/action/drive_on_heading_action.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

DriveOnHeadingAction::DriveOnHeadingAction(const std::string& xml_tag_name, const std::string& action_name,
                                           const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void DriveOnHeadingAction::initialize() {
  double dist;
  getInput("dist_to_travel", dist);
  double speed;
  getInput("speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  bool disable_collision_checks;
  getInput("disable_collision_checks", disable_collision_checks);

  // Populate the input message
  // goal_ is of type ActionT::Goal, which is
  // proto::DriveOnHeadingAction::Goal
  auto* target = goal_.mutable_target();
  target->set_x(dist);
  target->set_y(0.0);
  target->set_z(0.0);
  goal_.set_speed(speed);
  auto duration = commsgs::builtin_interfaces::Duration::FromSeconds(time_allowance);
  int64_t ns = duration.Nanoseconds();
  int32_t sec = static_cast<int32_t>(ns / 1'000'000'000LL);
  uint32_t nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  goal_.mutable_time_allowance()->mutable_stamp()->set_sec(sec);
  goal_.mutable_time_allowance()->mutable_stamp()->set_nanosec(nanosec);
  goal_.set_disable_collision_checks(disable_collision_checks);
}

void DriveOnHeadingAction::on_tick() {
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus DriveOnHeadingAction::on_success() {
  setOutput("error_code_id", static_cast<int32_t>(proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_NONE));
  setOutput("error_msg", std::string(""));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DriveOnHeadingAction::on_aborted() {
  if (result_.result) {
    setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
    setOutput("error_msg", result_.result->error_msg());
  } else {
    setOutput("error_code_id", static_cast<int32_t>(proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_UNKNOWN));
    setOutput("error_msg", std::string("Action aborted"));
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DriveOnHeadingAction::on_cancelled() {
  setOutput("error_code_id", static_cast<int32_t>(proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_NONE));
  setOutput("error_msg", std::string(""));
  return BT::NodeStatus::SUCCESS;
}

void DriveOnHeadingAction::on_timeout() {
  setOutput("error_code_id", static_cast<int32_t>(proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_TIMEOUT));
  setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::DriveOnHeadingAction>(
        name, "drive_on_heading", config);
  };

  factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::DriveOnHeadingAction>("DriveOnHeading",
                                                                                                 builder);
}
