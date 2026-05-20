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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @class BtActionServer
 * @brief Runs a behavior tree in-process (no ROS/autolink action server).
 */
template <class ActionT>
class BtActionServer
{
public:
    using Goal = typename ActionT::Goal;
    using Result = typename ActionT::Result;
    using Feedback = typename ActionT::Feedback;

    using OnGoalReceivedCallback =
        std::function<bool(std::shared_ptr<const Goal>)>;
    using OnLoopCallback = std::function<void()>;
    using OnPreemptCallback = std::function<void(std::shared_ptr<const Goal>)>;
    using OnCompletionCallback =
        std::function<void(std::shared_ptr<Result>, BtStatus)>;

    explicit BtActionServer(const std::string& name,
                            const std::vector<std::string>& plugin_lib_names,
                            const std::string& default_bt_xml_filename,
                            const std::string& plugin_lib_path,
                            OnGoalReceivedCallback on_goal_received_callback,
                            OnLoopCallback on_loop_callback,
                            OnPreemptCallback on_preempt_callback,
                            OnCompletionCallback on_completion_callback);

    ~BtActionServer();

    void SetGrootMonitoring(const bool enable, const unsigned server_port);

    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    void SetLoopDuration(std::chrono::milliseconds bt_loop_duration) {
        bt_loop_duration_ = bt_loop_duration;
    }

    BT::Blackboard::Ptr GetBlackboard() const {
        return blackboard_;
    }

    std::string GetCurrentBTFilename() const {
        return current_bt_xml_filename_;
    }

    std::string GetDefaultBTFilename() const {
        return default_bt_xml_filename_;
    }

    /** Run the loaded behavior tree for @p goal until done or canceled. */
    BtStatus Run(std::shared_ptr<const Goal> goal,
                 std::function<bool()> is_canceling = {});

    void RequestCancel() {
        cancel_requested_.store(true);
    }

    bool IsCancelRequested() const {
        return cancel_requested_.load();
    }

    void SetPendingGoal(std::shared_ptr<const Goal> goal) {
        pending_goal_ = std::move(goal);
        preempt_requested_.store(pending_goal_ != nullptr);
    }

    std::shared_ptr<const Goal> AcceptPendingGoal() {
        if (!pending_goal_) {
            return nullptr;
        }
        current_goal_ = pending_goal_;
        pending_goal_.reset();
        preempt_requested_.store(false);
        return current_goal_;
    }

    void TerminatePendingGoal() {
        pending_goal_.reset();
        preempt_requested_.store(false);
    }

    std::shared_ptr<const Goal> GetCurrentGoal() const {
        return current_goal_;
    }

    std::shared_ptr<const Goal> GetPendingGoal() const {
        return pending_goal_;
    }

    bool IsPreemptRequested() const {
        return preempt_requested_.load();
    }

    /** Feedback hook (no middleware publisher). */
    void PublishFeedback(std::shared_ptr<Feedback> /*feedback*/) {}

    const BT::Tree& GetTree() const {
        return tree_;
    }

    void HaltTree() {
        if (tree_.rootNode()) {
            tree_.haltTree();
        }
    }

    void SetInternalError(uint16_t error_code, const std::string& error_msg);

    void ResetInternalError();

    bool PopulateInternalError(std::shared_ptr<Result> result);

protected:
    void PopulateErrorCode(std::shared_ptr<Result> result);

    void CleanErrorCodes();

    std::string name_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;
    std::unique_ptr<BehaviorTreeEngine> bt_;
    std::vector<std::string> plugin_lib_names_;
    std::vector<std::string> error_code_name_prefixes_;

    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds default_server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    bool always_reload_bt_xml_ = false;
    bool enable_groot_monitoring_ = true;
    int groot_server_port_ = 1667;

    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;

    uint16_t internal_error_code_;
    std::string internal_error_msg_;

    std::shared_ptr<const Goal> current_goal_;
    std::shared_ptr<const Goal> pending_goal_;
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> preempt_requested_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "autonomy/tasks/behavior_tree/behavior_tree_action_server_impl.hpp"  // NOLINT(build/include_order)
