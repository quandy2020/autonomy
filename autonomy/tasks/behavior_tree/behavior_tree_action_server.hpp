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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/json_export.h"

#include "autolink/autolink.hpp"
#include "autonomy/common/simple_action_server.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @class BtActionServer
 * @brief An action server that uses behavior tree to execute an action
 *        (design aligned with nav2_behavior_tree::BtActionServer)
 */
template <class ActionT>
class BtActionServer
{
public:
    using ActionServer = autonomy::common::SimpleActionServer<ActionT>;

    using OnGoalReceivedCallback = std::function<bool(std::shared_ptr<const typename ActionT::Goal>)>;
    using OnLoopCallback = std::function<void()>;
    using OnPreemptCallback = std::function<void(std::shared_ptr<const typename ActionT::Goal>)>;
    using OnCompletionCallback = std::function<void(std::shared_ptr<typename ActionT::Result>, BtStatus)>;

    /**
     * @brief A constructor for nav2_behavior_tree::BtActionServer class
     */
    explicit BtActionServer(const std::shared_ptr<autolink::Node>& parent, const std::string& action_name,
                            const std::vector<std::string>& plugin_lib_names,
                            const std::string& default_bt_xml_filename,
                            OnGoalReceivedCallback on_goal_received_callback, OnLoopCallback on_loop_callback,
                            OnPreemptCallback on_preempt_callback, OnCompletionCallback on_completion_callback);

    /**
     * @brief A destructor for nav2_behavior_tree::BtActionServer class
     */
    ~BtActionServer();

    /**
     * @brief Enable (or disable) Groot2 monitoring of BT
     * @param enable Groot2 monitoring
     * @param server_port Groot2 Server port, first of the pair (server_port,
     * publisher_port)
     */
    void SetGrootMonitoring(const bool enable, const unsigned server_port);

    /**
     * @brief Replace current BT with another one
     * @param bt_xml_filename The file containing the new BT, uses default
     * filename if empty
     * @return bool true if the resulting BT correspond to the one in
     * bt_xml_filename. false if something went wrong, and previous BT is
     * maintained
     */
    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    /**
     * @brief Getter function for BT Blackboard
     * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
     */
    BT::Blackboard::Ptr GetBlackboard() const {
        return blackboard_;
    }

    /**
     * @brief Getter function for current BT XML filename
     * @return string Containing current BT XML filename
     */
    std::string GetCurrentBTFilename() const {
        return current_bt_xml_filename_;
    }

    /**
     * @brief Getter function for default BT XML filename
     */
    std::string GetDefaultBTFilename() const {
        return default_bt_xml_filename_;
    }

    /**
     * @brief Wrapper to accept pending goal if a preempt has been requested
     */
    const std::shared_ptr<const typename ActionT::Goal> AcceptPendingGoal() {
        return action_server_ ? action_server_->AcceptPendingGoal() : nullptr;
    }

    /**
     * @brief Wrapper to terminate pending goal if a preempt has been requested
     */
    void TerminatePendingGoal() {
        if (action_server_) {
            action_server_->TerminatePendingGoal();
        }
    }

    /**
     * @brief Wrapper to get current goal
     */
    const std::shared_ptr<const typename ActionT::Goal> GetCurrentGoal() const {
        return action_server_ ? action_server_->GetCurrentGoal() : nullptr;
    }

    /**
     * @brief Wrapper to get pending goal
     */
    const std::shared_ptr<const typename ActionT::Goal> GetPendingGoal() const {
        return action_server_ ? action_server_->GetPendingGoal() : nullptr;
    }

    /**
     * @brief Wrapper to publish action feedback
     */
    void PublishFeedback(std::shared_ptr<typename ActionT::Feedback> feedback) {
        if (action_server_) {
            action_server_->PublishFeedback(feedback);
        }
    }

    /**
     * @brief Getter function for the current BT tree
     * @return BT::Tree Current behavior tree
     */
    const BT::Tree& GetTree() const {
        return tree_;
    }

    /**
     * @brief Function to halt the current tree. It will interrupt the execution
     * of RUNNING nodes by calling their halt() implementation (only for Async
     * nodes that may return RUNNING) This should already done for all the exit
     * states of the action but preemption
     */
    void HaltTree() {
        tree_.haltTree();
    }

    /**
     * @brief Set internal error code and message
     * @param error_code the internal error code
     * @param error_msg the internal error message
     */
    void SetInternalError(uint16_t error_code, const std::string& error_msg);

    /**
     * @brief reset internal error code and message
     */
    void ResetInternalError();

    /**
     * @brief populate result with internal error code and error_msg if not NONE
     * @param result the action server result to be updated
     * @return bool action server result was changed
     */
    bool PopulateInternalError(typename std::shared_ptr<typename ActionT::Result> result);

protected:
    /**
     * @brief Action server callback
     */
    void ExecuteCallback();

    /**
     * @brief updates the action server result to the highest priority error
     * code posted on the blackboard
     * @param result the action server result to be updated
     */
    void PopulateErrorCode(typename std::shared_ptr<typename ActionT::Result> result);

    /**
     * @brief Setting BT error codes to success. Used to clean blackboard
     * between different BT runs
     */
    void CleanErrorCodes();

    // Action name
    std::string action_name_;

    // Parent node for creating the action server
    std::shared_ptr<autolink::Node> node_;

    // Action server (design aligned with nav2_util::SimpleActionServer)
    std::shared_ptr<ActionServer> action_server_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;

    // The XML file that contains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // The wrapper class for the BT functionality
    std::unique_ptr<BehaviorTreeEngine> bt_;

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // Error code name prefixes
    std::vector<std::string> error_code_name_prefixes_;

    // // A regular, non-spinning ROS node that we can use for calls to the
    // action client rclcpp::Node::SharedPtr client_node_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;

    // should the BT be reloaded even if the same xml filename is requested?
    bool always_reload_bt_xml_ = false;

    // Parameters for Groot2 monitoring
    bool enable_groot_monitoring_ = true;
    int groot_server_port_ = 1667;

    // User-provided callbacks
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;

    // internal error tracking (IOW not behaviorTree blackboard errors)
    uint16_t internal_error_code_;
    std::string internal_error_msg_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "autonomy/tasks/behavior_tree/behavior_tree_action_server_impl.hpp"  // NOLINT(build/include_order)