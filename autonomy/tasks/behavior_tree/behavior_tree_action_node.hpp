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
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/time/clock.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
template <class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
    /**
     * @brief A autonomy::tasks::behavior_tree::BtActionNode constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    BtActionNode(const std::string& xml_tag_name,
                 const std::string& action_name,
                 const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf),
          action_name_(action_name),
          should_send_goal_(true) {
        node_ =
            config().blackboard->template get<std::shared_ptr<autolink::Node>>(
                "node");

        // Get the required items from the blackboard
        auto bt_loop_duration =
            config().blackboard->template get<std::chrono::milliseconds>(
                "bt_loop_duration");
        if (!GetInputPortOrBlackboard(*this, *config().blackboard,
                                      "server_timeout", server_timeout_)) {
            server_timeout_ = std::chrono::milliseconds(10);  // Default timeout
        }
        wait_for_service_timeout_ =
            config().blackboard->template get<std::chrono::milliseconds>(
                "wait_for_service_timeout");

        // timeout should be less than bt_loop_duration to be able to finish the
        // current tick
        max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            bt_loop_duration * 0.5);

        // Initialize the input and output messages
        goal_ = typename ActionT::Goal();
        result_ = typename autolink::action::ClientGoalHandle<
            ActionT>::WrappedResult();

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        createActionClient(action_name_);

        // Give the derive class a chance to do any initialization
        ADEBUG << xml_tag_name.c_str() << " BtActionNode initialized";
    }

    BtActionNode() = delete;

    virtual ~BtActionNode() = default;

    /**
     * @brief Create instance of an action client
     * @param action_name Action name to create client for
     */
    void createActionClient(const std::string& action_name) {
        // Now that we have the autolink node to use, create the action client
        // for this BT action
        action_client_ =
            autolink::action::CreateClient<ActionT>(node_, action_name);

        // Make sure the server is actually there before continuing
        ADEBUG << "Waiting for \"" << action_name << "\" action server";
        int wait_count = 0;
        int max_wait = wait_for_service_timeout_.count() /
                       100;  // Convert ms to count (assuming 100ms intervals)
        while (!action_client_->ActionServerIsReady() &&
               wait_count < max_wait) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        if (!action_client_->ActionServerIsReady()) {
            AERROR << "\"" << action_name
                   << "\" action server not available after waiting for "
                   << wait_for_service_timeout_.count() / 1000.0 << "s";
            throw std::runtime_error(std::string("Action server ") +
                                     action_name + " not available");
        }
    }

    /**
     * @brief Any subclass of BtActionNode that accepts parameters must provide
     * a providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action server name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into
    // the processing for the action: on_tick, on_wait_for_result, and
    // on_success

    /**
     * @brief Function to perform some user-defined operation on tick
     * Could do dynamic checks, such as getting updates to values on the
     * blackboard
     */
    virtual void on_tick() {}

    /**
     * @brief Function to perform some user-defined operation after a timeout
     * waiting for a result that hasn't been received yet. Also provides access
     * to the latest feedback message from the action server. Feedback will be
     * nullptr in subsequent calls to this function if no new feedback is
     * received while waiting for a result.
     * @param feedback shared_ptr to latest feedback message, nullptr if no new
     * feedback was received
     */
    virtual void on_wait_for_result(
        std::shared_ptr<const typename ActionT::Feedback> /*feedback*/) {}

    /**
     * @brief Function to perform some user-defined operation upon successful
     * completion of the action. Could put a value on the blackboard.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override
     * return another value
     */
    virtual BT::NodeStatus on_success() {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Function to perform some user-defined operation when the action is
     * aborted.
     * @return BT::NodeStatus Returns FAILURE by default, user may override
     * return another value
     */
    virtual BT::NodeStatus on_aborted() {
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Function to perform some user-defined operation when the action is
     * cancelled.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override
     * return another value
     */
    virtual BT::NodeStatus on_cancelled() {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Function to perform work in a BT Node when the action server times
     * out Such as setting the error code ID status to timed out for action
     * clients.
     */
    virtual void on_timeout() {
        return;
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override {
        // first step to be done only at the beginning of the Action
        if (!BT::isStatusActive(status())) {
            // reset the flag to send the goal or not, allowing the user the
            // option to set it in on_tick
            should_send_goal_ = true;

            // Clear the input and output messages to make sure we have no
            // leftover from previous calls
            goal_ = typename ActionT::Goal();
            result_ = typename autolink::action::ClientGoalHandle<
                ActionT>::WrappedResult();

            // user defined callback, may modify "should_send_goal_".
            on_tick();

            // setting the status to RUNNING to notify the BT Loggers (if any)
            setStatus(BT::NodeStatus::RUNNING);

            if (!should_send_goal_) {
                return BT::NodeStatus::FAILURE;
            }
            send_new_goal();
        }

        try {
            // if new goal was sent and action server has not yet responded
            // check the future goal handle
            if (future_goal_handle_) {
                auto now = autolink::Clock::Now();
                auto elapsed_ns =
                    now.ToNanosecond() - time_goal_sent_.ToNanosecond();
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::nanoseconds(elapsed_ns));
                if (!is_future_goal_handle_complete(elapsed)) {
                    // return RUNNING if there is still some time before timeout
                    // happens
                    if (elapsed < server_timeout_) {
                        return BT::NodeStatus::RUNNING;
                    }
                    // if server has taken more time than the specified timeout
                    // value return FAILURE
                    LOG(WARNING)
                        << "Timed out while waiting for action server to "
                           "acknowledge goal request for "
                        << action_name_;
                    future_goal_handle_.reset();
                    on_timeout();
                    return BT::NodeStatus::FAILURE;
                }
            }

            // The following code corresponds to the "RUNNING" loop
            if (!goal_result_available_) {
                // user defined callback. May modify the value of
                // "goal_updated_"
                on_wait_for_result(feedback_);

                // reset feedback to avoid stale information
                feedback_.reset();

                if (goal_handle_) {
                    auto goal_status =
                        static_cast<autolink::action::GoalStatus>(
                            goal_handle_->GetStatus());
                    if (goal_updated_ &&
                        (goal_status ==
                             autolink::action::GoalStatus::EXECUTING ||
                         goal_status ==
                             autolink::action::GoalStatus::ACCEPTED)) {
                        goal_updated_ = false;
                        send_new_goal();
                        auto now = autolink::Clock::Now();
                        auto elapsed_ns =
                            now.ToNanosecond() - time_goal_sent_.ToNanosecond();
                        auto elapsed = std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::nanoseconds(elapsed_ns));
                        if (!is_future_goal_handle_complete(elapsed)) {
                            if (elapsed < server_timeout_) {
                                return BT::NodeStatus::RUNNING;
                            }
                            LOG(WARNING) << "Timed out while waiting for "
                                            "action server to "
                                            "acknowledge goal request for "
                                         << action_name_;
                            future_goal_handle_.reset();
                            on_timeout();
                            return BT::NodeStatus::FAILURE;
                        }
                    }
                }

                // check if we finally received the result
                if (!goal_result_available_) {
                    // Yield this Action, returning RUNNING
                    return BT::NodeStatus::RUNNING;
                }
            }
        } catch (const std::runtime_error& e) {
            if (e.what() == std::string("send_goal failed") ||
                e.what() ==
                    std::string("Goal was rejected by the action server")) {
                // Action related failure that should not fail the tree, but the
                // node
                return BT::NodeStatus::FAILURE;
            } else {
                // Internal exception to propagate to the tree
                throw e;
            }
        }

        BT::NodeStatus bt_status;
        switch (result_.code) {
            case autolink::action::ResultCode::SUCCEEDED:
                bt_status = on_success();
                break;

            case autolink::action::ResultCode::ABORTED:
                bt_status = on_aborted();
                break;

            case autolink::action::ResultCode::CANCELED:
                bt_status = on_cancelled();
                break;

            default:
                throw std::logic_error(
                    "BtActionNode::Tick: invalid status value");
        }

        goal_handle_.reset();
        return bt_status;
    }

    /**
     * @brief The other (optional) override required by a BT action. In this
     * case, we make sure to cancel the action if it is still running.
     */
    void halt() override {
        if (should_cancel_goal()) {
            auto future_result = action_client_->AsyncGetResult(goal_handle_);
            auto future_cancel = action_client_->AsyncCancelGoal(goal_handle_);
            auto cancel_status = future_cancel.wait_for(server_timeout_);
            if (cancel_status != std::future_status::ready) {
                AERROR << "Failed to cancel action server for " << action_name_;
            } else {
                ADEBUG << "Cancelled action server for " << action_name_;
            }
            auto result_status = future_result.wait_for(server_timeout_);
            if (result_status != std::future_status::ready) {
                AERROR << "Failed to get result for " << action_name_
                       << " in node halt!";
            } else {
                ADEBUG << "Got result for " << action_name_ << " in node halt!";
            }
            on_cancelled();
        }
        resetStatus();
    }

protected:
    /**
     * @brief Function to check if current goal should be cancelled
     * @return bool True if current goal should be cancelled, false otherwise
     */
    bool should_cancel_goal() {
        // Shut the node down if it is currently running
        if (status() != BT::NodeStatus::RUNNING) {
            return false;
        }

        // No need to cancel the goal if goal handle is invalid
        if (!goal_handle_) {
            return false;
        }

        auto goal_status = static_cast<autolink::action::GoalStatus>(
            goal_handle_->GetStatus());

        // Check if the goal is still executing
        return goal_status == autolink::action::GoalStatus::ACCEPTED ||
               goal_status == autolink::action::GoalStatus::EXECUTING;
    }

    /**
     * @brief Function to send new goal to action server
     */
    void send_new_goal() {
        goal_result_available_ = false;
        typename autolink::action::Client<ActionT>::SendGoalOptions
            send_goal_options;
        send_goal_options
            .result_callback = [this](
                                   const typename autolink::action::
                                       ClientGoalHandle<ActionT>::WrappedResult&
                                           result) {
            if (future_goal_handle_ && future_goal_handle_->valid()) {
                ADEBUG
                    << "Goal result for " << action_name_
                    << " available, but it hasn't received the goal response "
                       "yet. "
                    << "It's probably a goal result for the last goal request";
                return;
            }

            // if goal ids are not matched, the older goal call this callback so
            // ignore the result if matched, it must be processed (including
            // aborted)
            if (this->goal_handle_ &&
                this->goal_handle_->GetGoalId() == result.goal_id) {
                goal_result_available_ = true;
                result_.goal_id = result.goal_id;
                result_.code = result.code;
                result_.result = result.result;
                emitWakeUpSignal();
            }
        };
        send_goal_options.feedback_callback =
            [this](std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>>,
                   const std::shared_ptr<const typename ActionT::Feedback>
                       feedback) {
                feedback_ = feedback;
                emitWakeUpSignal();
            };

        future_goal_handle_ = std::make_shared<std::shared_future<
            std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>>>>(
            action_client_->AsyncSendGoal(goal_, send_goal_options));
        time_goal_sent_ = autolink::Clock::Now();
    }

    /**
     * @brief Function to check if the action server acknowledged a new goal
     * @param elapsed Duration since the last goal was sent and future goal
     * handle has not completed. After waiting for the future to complete, this
     * value is incremented with the timeout value.
     * @return boolean True if future_goal_handle_ returns SUCCESS, False
     * otherwise
     */
    bool is_future_goal_handle_complete(std::chrono::milliseconds& elapsed) {
        if (!future_goal_handle_ || !future_goal_handle_->valid()) {
            return false;
        }

        auto remaining = server_timeout_ - elapsed;

        // server has already timed out, no need to sleep
        if (remaining <= std::chrono::milliseconds(0)) {
            future_goal_handle_.reset();
            return false;
        }

        auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
        auto result = future_goal_handle_->wait_for(timeout);
        elapsed += timeout;

        if (result == std::future_status::deferred) {
            future_goal_handle_.reset();
            throw std::runtime_error("send_goal failed");
        }

        if (result == std::future_status::ready) {
            goal_handle_ = future_goal_handle_->get();
            future_goal_handle_.reset();
            if (!goal_handle_) {
                throw std::runtime_error(
                    "Goal was rejected by the action server");
            }
            return true;
        }

        return false;
    }

    /**
     * @brief Function to increment recovery count on blackboard if this node
     * wraps a recovery
     */
    void increment_recovery_count() {
        int recovery_count = 0;
        [[maybe_unused]] auto res = config().blackboard->get(
            "number_recoveries", recovery_count);  // NOLINT
        recovery_count += 1;
        config().blackboard->set("number_recoveries",
                                 recovery_count);  // NOLINT
    }

    std::string action_name_;
    std::shared_ptr<::autolink::action::Client<ActionT>> action_client_;

    // All ROS2 actions have a goal and a result
    typename ActionT::Goal goal_;
    bool goal_updated_{false};
    bool goal_result_available_{false};
    std::shared_ptr<::autolink::action::ClientGoalHandle<ActionT>> goal_handle_;
    typename autolink::action::ClientGoalHandle<ActionT>::WrappedResult result_;

    // To handle feedback from action server
    std::shared_ptr<const typename ActionT::Feedback> feedback_;

    // The node that will be used for any ROS operations
    std::shared_ptr<::autolink::Node> node_;

    // The timeout value while waiting for response from a server when a
    // new action goal is sent or canceled
    std::chrono::milliseconds server_timeout_;

    // The timeout value for BT loop execution
    std::chrono::milliseconds max_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;

    // To track the action server acknowledgement when a new goal is sent
    std::shared_ptr<std::shared_future<
        std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>>>>
        future_goal_handle_;
    autolink::Time time_goal_sent_;

    // Can be set in on_tick or on_wait_for_result to indicate if a goal should
    // be sent.
    bool should_send_goal_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy