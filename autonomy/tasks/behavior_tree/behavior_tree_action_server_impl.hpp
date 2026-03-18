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

#pragma once

#include <fstream>
#include <sstream>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_server.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

template <class ActionT>
BtActionServer<ActionT>::BtActionServer(const std::shared_ptr<autolink::Node>& parent, const std::string& action_name,
                                        const std::vector<std::string>& plugin_lib_names,
                                        const std::string& default_bt_xml_filename,
                                        OnGoalReceivedCallback on_goal_received_callback,
                                        OnLoopCallback on_loop_callback, OnPreemptCallback on_preempt_callback,
                                        OnCompletionCallback on_completion_callback)
    : action_name_(action_name),
      default_bt_xml_filename_(default_bt_xml_filename),
      current_bt_xml_filename_(default_bt_xml_filename),
      plugin_lib_names_(plugin_lib_names),
      on_goal_received_callback_(on_goal_received_callback),
      on_loop_callback_(on_loop_callback),
      on_preempt_callback_(on_preempt_callback),
      on_completion_callback_(on_completion_callback),
      bt_loop_duration_(std::chrono::milliseconds(10)),
      default_server_timeout_(std::chrono::milliseconds(10)),
      wait_for_service_timeout_(std::chrono::milliseconds(1000)),
      internal_error_code_(0),
      internal_error_msg_("") {
  node_ = parent;

  // Create behavior tree engine
  bt_ = std::make_unique<BehaviorTreeEngine>(plugin_lib_names, parent);

  // Create blackboard and set keys required by BT nodes before tree is loaded
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", parent);                                         // NOLINT
  blackboard_->set("bt_loop_duration", bt_loop_duration_);                  // NOLINT
  blackboard_->set("wait_for_service_timeout", wait_for_service_timeout_);  // NOLINT

  // Action server: execute callback runs BT; optional goal callback rejects by content (nav2 does this inside
  // executeCallback)
  action_server_ = std::make_shared<ActionServer>(
      node_, action_name_, [this]() { ExecuteCallback(); }, nullptr, std::chrono::milliseconds(500), false);
  action_server_->SetGoalCallback(
      [this](const autolink::action::GoalUUID& /*uuid*/, std::shared_ptr<const typename ActionT::Goal> goal) {
        if (!goal) return autolink::action::GoalResponse::REJECT;
        bool accept = on_goal_received_callback_(goal);
        return accept ? autolink::action::GoalResponse::ACCEPT_AND_EXECUTE : autolink::action::GoalResponse::REJECT;
      });
}

template <class ActionT>
BtActionServer<ActionT>::~BtActionServer() {}

template <class ActionT>
void BtActionServer<ActionT>::SetGrootMonitoring(const bool enable, const unsigned server_port) {
  enable_groot_monitoring_ = enable;
  groot_server_port_ = server_port;
  // TODO: Implement Groot2 monitoring if needed
}

template <class ActionT>
bool BtActionServer<ActionT>::LoadBehaviorTree(const std::string& bt_xml_filename) {
  std::string filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  if (filename.empty()) {
    AERROR << "No behavior tree file specified";
    return false;
  }

  try {
    // Halt current tree if running
    if (tree_.rootNode()) {
      tree_.haltTree();
    }

    // Create new tree from file
    tree_ = bt_->CreateTreeFromFile(filename, blackboard_);
    current_bt_xml_filename_ = filename;

    return true;
  } catch (const std::exception& e) {
    AERROR << "Failed to load behavior tree from " << filename << ": " << e.what();
    return false;
  }
}

template <class ActionT>
void BtActionServer<ActionT>::SetInternalError(uint16_t error_code, const std::string& error_msg) {
  internal_error_code_ = error_code;
  internal_error_msg_ = error_msg;
}

template <class ActionT>
void BtActionServer<ActionT>::ResetInternalError() {
  internal_error_code_ = 0;
  internal_error_msg_ = "";
}

template <class ActionT>
bool BtActionServer<ActionT>::PopulateInternalError(typename std::shared_ptr<typename ActionT::Result> result) {
  if (internal_error_code_ != 0 && result) {
    // Set error code and message in result
    // Note: This depends on the protobuf structure
    // For now, we'll set it if the result has error_code and error_msg
    // fields
    return true;
  }
  return false;
}

template <class ActionT>
void BtActionServer<ActionT>::ExecuteCallback() {
  // Aligned with nav2: give server a chance to reject goal (we also reject via SetGoalCallback)
  if (action_server_ && !on_goal_received_callback_(action_server_->GetCurrentGoal())) {
    auto result = std::make_shared<typename ActionT::Result>();
    PopulateErrorCode(result);
    action_server_->TerminateCurrent(result);
    CleanErrorCodes();
    return;
  }

  if (!tree_.rootNode()) {
    AERROR << "Behavior tree not initialized";
    return;
  }

  CleanErrorCodes();
  ResetInternalError();

  auto is_canceling = [this]() {
    if (!action_server_) return true;
    if (!action_server_->IsServerActive()) return true;
    return action_server_->IsCancelRequested();
  };

  auto on_loop = [this]() {
    if (action_server_ && action_server_->IsPreemptRequested() && on_preempt_callback_) {
      on_preempt_callback_(action_server_->GetPendingGoal());
    }
    on_loop_callback_();
  };

  BtStatus status = bt_->Run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  // Ensure BT is not in running state from previous execution (nav2: haltAllActions)
  if (tree_.rootNode()) {
    tree_.haltTree();
  }

  auto result = std::make_shared<typename ActionT::Result>();
  PopulateErrorCode(result);
  PopulateInternalError(result);
  on_completion_callback_(result, status);

  switch (status) {
    case BtStatus::SUCCEEDED:
      action_server_->SucceededCurrent(result);
      break;
    case BtStatus::FAILED:
      action_server_->TerminateCurrent(result);
      break;
    case BtStatus::CANCELED:
      action_server_->TerminateAll(result);
      break;
  }

  CleanErrorCodes();
}

template <class ActionT>
void BtActionServer<ActionT>::PopulateErrorCode(typename std::shared_ptr<typename ActionT::Result> result) {
  // TODO: Extract error codes from blackboard and populate result
  // This depends on the specific action result structure
}

template <class ActionT>
void BtActionServer<ActionT>::CleanErrorCodes() {
  // TODO: Clean error codes from blackboard
  // This depends on the error code naming convention
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
