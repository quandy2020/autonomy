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

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_server.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

template <class ActionT>
BtActionServer<ActionT>::BtActionServer(
    const std::string& name,
    const std::vector<std::string>& plugin_lib_names,
    const std::string& default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback, OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback)
    : name_(name),
      default_bt_xml_filename_(default_bt_xml_filename),
      current_bt_xml_filename_(default_bt_xml_filename),
      plugin_lib_names_(plugin_lib_names),
      on_goal_received_callback_(std::move(on_goal_received_callback)),
      on_loop_callback_(std::move(on_loop_callback)),
      on_preempt_callback_(std::move(on_preempt_callback)),
      on_completion_callback_(std::move(on_completion_callback)),
      bt_loop_duration_(std::chrono::milliseconds(10)),
      default_server_timeout_(std::chrono::milliseconds(10)),
      wait_for_service_timeout_(std::chrono::milliseconds(1000)),
      internal_error_code_(0),
      internal_error_msg_("") {
    bt_ = std::make_unique<BehaviorTreeEngine>(plugin_lib_names);
    blackboard_ = BT::Blackboard::create();  // NOLINT
    blackboard_->set("bt_loop_duration", bt_loop_duration_);  // NOLINT
    blackboard_->set("wait_for_service_timeout", wait_for_service_timeout_);  // NOLINT
}

template <class ActionT>
BtActionServer<ActionT>::~BtActionServer() {
    HaltTree();
}

template <class ActionT>
void BtActionServer<ActionT>::SetGrootMonitoring(const bool enable,
                                                 const unsigned server_port) {
    enable_groot_monitoring_ = enable;
    groot_server_port_ = static_cast<int>(server_port);
}

template <class ActionT>
bool BtActionServer<ActionT>::LoadBehaviorTree(
    const std::string& bt_xml_filename) {
    std::string filename =
        bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    if (filename.empty()) {
        AERROR << "No behavior tree file specified";
        return false;
    }

    if (!always_reload_bt_xml_ && filename == current_bt_xml_filename_ &&
        tree_.rootNode()) {
        return true;
    }

    try {
        if (tree_.rootNode()) {
            tree_.haltTree();
        }
        tree_ = bt_->CreateTreeFromFile(filename, blackboard_);
        current_bt_xml_filename_ = filename;
        return true;
    } catch (const std::exception& e) {
        AERROR << "Failed to load behavior tree from " << filename << ": "
               << e.what();
        return false;
    }
}

template <class ActionT>
void BtActionServer<ActionT>::SetInternalError(uint16_t error_code,
                                               const std::string& error_msg) {
    internal_error_code_ = error_code;
    internal_error_msg_ = error_msg;
}

template <class ActionT>
void BtActionServer<ActionT>::ResetInternalError() {
    internal_error_code_ = 0;
    internal_error_msg_.clear();
}

template <class ActionT>
bool BtActionServer<ActionT>::PopulateInternalError(
    std::shared_ptr<Result> result) {
    if (internal_error_code_ != 0 && result) {
        using ErrorCode = decltype(result->error_code());
        result->set_error_code(static_cast<ErrorCode>(internal_error_code_));
        result->set_error_msg(internal_error_msg_);
        return true;
    }
    return false;
}

template <class ActionT>
BtStatus BtActionServer<ActionT>::Run(std::shared_ptr<const Goal> goal,
                                      std::function<bool()> is_canceling) {
    cancel_requested_.store(false);
    current_goal_ = goal;
    pending_goal_.reset();
    preempt_requested_.store(false);

    if (!goal || !on_goal_received_callback_(goal)) {
        auto result = std::make_shared<Result>();
        PopulateErrorCode(result);
        PopulateInternalError(result);
        if (on_completion_callback_) {
            on_completion_callback_(result, BtStatus::FAILED);
        }
        CleanErrorCodes();
        current_goal_.reset();
        return BtStatus::FAILED;
    }

    if (!tree_.rootNode()) {
        AERROR << "Behavior tree not initialized for " << name_;
        current_goal_.reset();
        return BtStatus::FAILED;
    }

    CleanErrorCodes();
    ResetInternalError();

    auto is_canceling_fn = [this, is_canceling]() {
        if (cancel_requested_.load()) {
            return true;
        }
        return is_canceling && is_canceling();
    };

    auto on_loop = [this]() {
        if (preempt_requested_.load() && pending_goal_ && on_preempt_callback_) {
            on_preempt_callback_(pending_goal_);
        }
        if (on_loop_callback_) {
            on_loop_callback_();
        }
    };

    BtStatus status =
        bt_->Run(&tree_, on_loop, is_canceling_fn, bt_loop_duration_);

    if (tree_.rootNode()) {
        tree_.haltTree();
    }

    auto result = std::make_shared<Result>();
    PopulateErrorCode(result);
    PopulateInternalError(result);
    if (on_completion_callback_) {
        on_completion_callback_(result, status);
    }
    CleanErrorCodes();
    current_goal_.reset();
    return status;
}

template <class ActionT>
void BtActionServer<ActionT>::PopulateErrorCode(std::shared_ptr<Result> result) {
    (void)result;
}

template <class ActionT>
void BtActionServer<ActionT>::CleanErrorCodes() {
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
