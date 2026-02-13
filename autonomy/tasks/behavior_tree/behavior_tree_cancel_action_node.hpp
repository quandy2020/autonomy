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
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/action_node.h"

#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action for cancelling BT node
 * @tparam ActionT Type of action
 */
template <class ActionT>
class BtCancelActionNode : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::BtCancelActionNode constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    BtCancelActionNode(const std::string& xml_tag_name, const std::string& action_name,
                       const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
        node_ = config().blackboard->template get<std::shared_ptr<::autolink::Node>>("node");

        // Get the required items from the blackboard
        if (!GetInputPortOrBlackboard(*this, *config().blackboard, "server_timeout", server_timeout_)) {
            server_timeout_ = std::chrono::milliseconds(10);  // Default timeout
        }
        wait_for_service_timeout_ =
            config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        createCancelClient(action_name_);

        // Give the derive class a chance to do any initialization
        ADEBUG << xml_tag_name.c_str() << " BtCancelActionNode initialized";
    }

    BtCancelActionNode() = delete;

    virtual ~BtCancelActionNode() {}

    /**
     * @brief Create instance of a cancel goal client
     * @param action_name Action name to create cancel client for
     */
    void createCancelClient(const std::string& action_name) {
        // Create cancel_goal service client directly
        cancel_goal_client_ =
            node_->CreateClient<autolink::action::internal::CancelGoalRequest,
                                autolink::action::internal::CancelGoalResponse>(action_name + "/cancel_goal");

        // Make sure the server is actually there before continuing
        ADEBUG << "Waiting for " << action_name.c_str() << " cancel_goal service";
        int wait_count = 0;
        int max_wait = wait_for_service_timeout_.count() / 100;  // Convert ms to count (assuming 100ms intervals)
        while (!cancel_goal_client_->ServiceIsReady() && wait_count < max_wait) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        if (!cancel_goal_client_->ServiceIsReady()) {
            AERROR << "\"" << action_name << "\" cancel_goal service not available after waiting for "
                   << wait_for_service_timeout_.count() / 1000.0 << "s";
            throw std::runtime_error(std::string("Cancel goal service ") + action_name + " not available");
        }
    }

    /**
     * @brief Any subclass of BtCancelActionNode that accepts parameters must
     * provide a providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {BT::InputPort<std::string>("server_name", "Action server name"),
                               BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    void halt() override {}

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // Send cancel request with empty goal_id to cancel all goals
        // Note: autolink cancel_goal service requires a goal_id, but we can
        // send an empty/zero UUID to cancel all goals (server-dependent
        // behavior)
        auto request = std::make_shared<autolink::action::internal::CancelGoalRequest>();
        // Leave goal_id as zero/empty to cancel all goals

        auto response = cancel_goal_client_->SendRequest(
            request, std::chrono::duration_cast<std::chrono::seconds>(server_timeout_));

        if (!response || response->goals_canceling == 0) {
            ADEBUG << "No goals were canceled for " << action_name_.c_str();
            // Return SUCCESS anyway, as it's not an error if there are no goals
            // to cancel
        }
        return BT::NodeStatus::SUCCESS;
    }

protected:
    std::string action_name_;
    std::shared_ptr<
        autolink::Client<autolink::action::internal::CancelGoalRequest, autolink::action::internal::CancelGoalResponse>>
        cancel_goal_client_;

    // The node that will be used for any autolink operations
    std::shared_ptr<::autolink::Node> node_;

    // The timeout value while waiting for response from a server when a
    // new action goal is canceled
    std::chrono::milliseconds server_timeout_;
    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy