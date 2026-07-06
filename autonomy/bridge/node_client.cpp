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

// Template definitions and explicit instantiations for NodeClient.
// Navigator action types are instantiated here so bridge handlers can link
// without pulling template bodies into every translation unit.

#include "autonomy/bridge/node_client.hpp"

#include <thread>

#include "autolink/action/create_client.hpp"
#include "autolink/state.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/proto/nav_msgs.pb.h"

namespace autonomy {
namespace bridge {

template <typename ActionT>
NodeClient<ActionT>::NodeClient(std::shared_ptr<autolink::Node> node,
                                  std::string action_name)
    : node_(std::move(node)), action_name_(std::move(action_name)) {
    client_ = autolink::action::CreateClient<ActionT>(node_, action_name_);
}

template <typename ActionT>
bool NodeClient<ActionT>::ActionServerIsReady() const {
    return client_ && client_->ActionServerIsReady();
}

template <typename ActionT>
bool NodeClient<ActionT>::WaitForServer(
    const std::chrono::milliseconds poll_interval,
    const std::chrono::milliseconds timeout) {
    // timeout == 0: no upper bound; exit only on readiness or autolink shutdown.
    const auto deadline = timeout.count() > 0
                              ? std::chrono::steady_clock::now() + timeout
                              : std::chrono::steady_clock::time_point::max();

    while (autolink::OK()) {
        if (ActionServerIsReady()) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            AWARN << action_name_ << ": action server not ready before timeout.";
            return false;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return false;
}

template <typename ActionT>
std::shared_future<std::shared_ptr<typename NodeClient<ActionT>::GoalHandle>>
NodeClient<ActionT>::AsyncSendGoal(const Goal& goal,
                                   const SendGoalOptions& options) {
    return client_->AsyncSendGoal(goal, options);
}

template <typename ActionT>
std::shared_future<typename NodeClient<ActionT>::WrappedResult>
NodeClient<ActionT>::AsyncGetResult(std::shared_ptr<GoalHandle> goal_handle) {
    return client_->AsyncGetResult(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> NodeClient<ActionT>::AsyncCancelGoal(
    std::shared_ptr<GoalHandle> goal_handle) {
    return client_->AsyncCancelGoal(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> NodeClient<ActionT>::AsyncCancelAllGoals() {
    return client_->AsyncCancelAllGoals();
}

template <typename ActionT>
std::optional<typename NodeClient<ActionT>::WrappedResult>
NodeClient<ActionT>::SendGoalAndWait(
    const Goal& goal, const SendGoalOptions& options,
    const std::chrono::milliseconds accept_timeout,
    const std::chrono::milliseconds result_timeout) {
    // Phase 1: wait for server to accept or reject the goal.
    const auto accepted_future = AsyncSendGoal(goal, options);
    if (accepted_future.wait_for(accept_timeout) != std::future_status::ready) {
        AERROR << action_name_ << ": timeout waiting for goal acceptance.";
        return std::nullopt;
    }

    const auto goal_handle = accepted_future.get();
    if (!goal_handle) {
        AWARN << action_name_ << ": goal rejected by action server.";
        return std::nullopt;
    }

    // Phase 2: autolink client already registered result polling in AsyncSendGoal;
    // reuse the handle's shared future for the terminal WrappedResult.
    auto result_future = goal_handle->AsyncGetResult();
    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
        AERROR << action_name_ << ": timeout waiting for action result.";
        return std::nullopt;
    }

    try {
        return result_future.get();
    } catch (const std::exception& e) {
        AERROR << action_name_ << ": failed to get action result: " << e.what();
        return std::nullopt;
    }
}

// Navigator action servers (see bridge/constants.hpp for action names).
template class NodeClient<commsgs::proto::nav_msgs::NavigateToPoseAction>;
template class NodeClient<commsgs::proto::nav_msgs::NavigateThroughPosesAction>;

}  // namespace bridge
}  // namespace autonomy
