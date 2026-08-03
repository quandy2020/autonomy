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
#include <memory>
#include <optional>
#include <string>

#include "autolink/action/client.hpp"
#include "autolink/node/node.hpp"

namespace autonomy {
namespace bridge {

/**
 * @brief Bridge-side template wrapper for autolink::action::Client.
 *
 * gRPC / MQTT handlers use NodeClient to call in-process action servers
 * (e.g. navigator BtActionServer on navigate_to_pose). The action_name
 * must match the server registration; see constants.hpp.
 *
 * Async methods delegate to autolink::action::Client. SendGoalAndWait is a
 * convenience helper for unary-style bridge RPC handlers.
 *
 * @tparam ActionT Protobuf action type with nested Goal, Feedback, Result
 *                 (e.g. automsgs::actions::NavigateToPoseAction).
 */
template <typename ActionT>
class NodeClient
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using AutolinkClient = autolink::action::Client<ActionT>;
    using GoalHandle = autolink::action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;
    using SendGoalOptions = typename AutolinkClient::SendGoalOptions;
    using SharedPtr = std::shared_ptr<NodeClient<ActionT>>;

    /**
     * @param node Shared autolink node used to create service clients/readers.
     * @param action_name Action server name (ROS 2 style, no /send_goal suffix).
     */
    NodeClient(std::shared_ptr<autolink::Node> node, std::string action_name);

    /** @brief Registered action server name passed at construction. */
    const std::string& action_name() const { return action_name_; }

    /** @brief Underlying autolink node. */
    std::shared_ptr<autolink::Node> node() const { return node_; }

    /** @brief Low-level autolink action client (for advanced use). */
    std::shared_ptr<AutolinkClient> client() const { return client_; }

    /**
     * @brief True when send_goal / cancel_goal / get_result endpoints exist.
     */
    bool ActionServerIsReady() const;

    /**
     * @brief Poll until the action server is ready or timeout elapses.
     *
     * @param poll_interval Sleep between readiness checks.
     * @param timeout Zero means wait until autolink shuts down.
     * @return true if all action services became ready in time.
     */
    bool WaitForServer(
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100),
        std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

    /**
     * @brief Send a goal asynchronously.
     *
     * Completes when the server accepts or rejects. Use goal_response_callback,
     * feedback_callback, and result_callback in options for streaming updates.
     */
    std::shared_future<std::shared_ptr<GoalHandle>> AsyncSendGoal(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions());

    /** @brief Poll terminal result for an accepted goal handle. */
    std::shared_future<WrappedResult> AsyncGetResult(
        std::shared_ptr<GoalHandle> goal_handle);

    /** @brief Request cancellation of a single active goal. */
    std::shared_future<bool> AsyncCancelGoal(
        std::shared_ptr<GoalHandle> goal_handle);

    /** @brief Request cancellation of all goals on the server (zero UUID). */
    std::shared_future<bool> AsyncCancelAllGoals();

    /**
     * @brief Send a goal and block until a terminal result or timeout.
     *
     * Suitable for bridge handlers that map one RPC to one action lifecycle.
     * Returns std::nullopt on acceptance timeout, rejection, result timeout,
     * or client error.
     */
    std::optional<WrappedResult> SendGoalAndWait(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions(),
        std::chrono::milliseconds accept_timeout = std::chrono::seconds(30),
        std::chrono::milliseconds result_timeout = std::chrono::seconds(300));

private:
    std::shared_ptr<autolink::Node> node_;
    std::string action_name_;
    std::shared_ptr<AutolinkClient> client_;
};

}  // namespace bridge
}  // namespace autonomy
