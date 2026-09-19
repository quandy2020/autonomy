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

/**
 * @file node_client.hpp
 * @brief Bridge-side wrapper for autolink::action::Client (async + Await).
 *
 * @tparam ActionT Declared on the class; protobuf action with Goal /
 * Feedback / Result nested types.
 *
 * @see NodeClient
 */

#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "autolink/action/client.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {

/**
 * @brief Bridge-side wrapper for autolink::action::Client.
 *
 * Handlers call in-process action servers (e.g. navigate_to_pose).
 * Async APIs return futures; @ref AwaitGoalResult covers unary-style RPCs.
 *
 * @tparam ActionT Protobuf action with Goal / Feedback / Result.
 */
template <typename ActionT>
class NodeClient
{
public:
    /** @brief Action goal protobuf type (`ActionT::Goal`). */
    using Goal = typename ActionT::Goal;
    /** @brief Action feedback protobuf type (`ActionT::Feedback`). */
    using Feedback = typename ActionT::Feedback;
    /** @brief Action result protobuf type (`ActionT::Result`). */
    using Result = typename ActionT::Result;
    /** @brief Underlying Autolink action client type. */
    using AutolinkClient = autolink::action::Client<ActionT>;
    /** @brief Accepted-goal handle type from the Autolink client. */
    using GoalHandle = autolink::action::ClientGoalHandle<ActionT>;
    /** @brief Terminal result wrapper (`GoalHandle::WrappedResult`). */
    using WrappedResult = typename GoalHandle::WrappedResult;
    /** @brief Options / callbacks passed to SendGoal. */
    using SendGoalOptions = typename AutolinkClient::SendGoalOptions;

    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(NodeClient<ActionT>)

    /**
     * @brief Construct a NodeClient bound to an action server name.
     *
     * @param[in] node        Shared autolink node for service clients/readers.
     * @param[in] action_name Action server name (no /send_goal suffix).
     */
    NodeClient(std::shared_ptr<autolink::Node> node, std::string action_name);

    /**
     * @brief Return the configured action server name.
     *
     * @return Action name string.
     */
    const std::string& GetActionName() const { return action_name_; }

    /**
     * @brief Return the shared autolink node.
     *
     * @return Shared pointer to the node.
     */
    std::shared_ptr<autolink::Node> GetNode() const { return node_; }

    /**
     * @brief Return the underlying autolink action client.
     *
     * @return Shared pointer to AutolinkClient.
     */
    std::shared_ptr<AutolinkClient> GetClient() const { return client_; }

    /**
     * @brief True when send_goal / cancel_goal / get_result are available.
     *
     * @return Whether the action server is ready.
     */
    bool CheckServerReady() const;

    /**
     * @brief Poll until the action server is ready or timeout elapses.
     *
     * @param[in] poll_interval Sleep between readiness checks.
     * @param[in] timeout       Zero waits until autolink shutdown.
     * @return                  true if the server became ready before timeout.
     */
    bool WaitServer(
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100),
        std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

    /**
     * @brief Send a goal; completes when accepted or rejected.
     *
     * @param[in] goal    Goal message to send.
     * @param[in] options Optional send-goal callbacks and flags.
     * @return            Future that resolves to an accepted goal handle (or null).
     */
    std::shared_future<std::shared_ptr<GoalHandle>> SendGoal(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions());

    /**
     * @brief Poll terminal result for an accepted goal handle.
     *
     * @param[in] goal_handle Accepted goal handle.
     * @return                Future that resolves to the wrapped terminal result.
     */
    std::shared_future<WrappedResult> GetResult(
        std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Cancel one active goal.
     *
     * @param[in] goal_handle Goal handle to cancel.
     * @return                Future that resolves to whether cancel was accepted.
     */
    std::shared_future<bool> CancelGoal(std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Cancel all goals on the server (zero UUID).
     *
     * @return Future that resolves to whether cancel-all was accepted.
     */
    std::shared_future<bool> CancelAllGoals();

    /**
     * @brief Send a goal and block until a terminal result or timeout.
     *
     * @param[in] goal           Goal message to send.
     * @param[in] options        Optional send-goal callbacks and flags.
     * @param[in] accept_timeout Max wait for accept/reject.
     * @param[in] result_timeout Max wait for terminal result.
     * @return                   nullopt on accept timeout, rejection, result timeout, or error.
     */
    std::optional<WrappedResult> AwaitGoalResult(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions(),
        std::chrono::milliseconds accept_timeout = std::chrono::seconds(30),
        std::chrono::milliseconds result_timeout = std::chrono::seconds(300));

private:
    /**
     * @brief Shared Autolink node used to create the action client.
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Action server name (no `/send_goal` suffix).
     */
    std::string action_name_;

    /**
     * @brief Owned Autolink action client bound to @ref action_name_.
     */
    std::shared_ptr<AutolinkClient> client_{nullptr};
};

}  // namespace bridge
}  // namespace autonomy
