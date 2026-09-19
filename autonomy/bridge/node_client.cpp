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
 * @file node_client.cpp
 * @brief Template definitions and explicit instantiations for NodeClient.
 */

#include "autonomy/bridge/node_client.hpp"

#include <thread>

#include "autolink/action/create_client.hpp"
#include "autolink/common/log.hpp"
#include "autolink/state.hpp"
#include <automsgs/actions/nav_actions.pb.h>

namespace autonomy {
namespace bridge {

template <typename ActionT>
NodeClient<ActionT>::NodeClient(std::shared_ptr<autolink::Node> node,
                                std::string action_name)
    : node_(std::move(node)), action_name_(std::move(action_name)) {
    client_ = autolink::action::CreateClient<ActionT>(node_, action_name_);
}

template <typename ActionT>
bool NodeClient<ActionT>::CheckServerReady() const {
    return client_ && client_->ActionServerIsReady();
}

template <typename ActionT>
bool NodeClient<ActionT>::WaitServer(
    const std::chrono::milliseconds poll_interval,
    const std::chrono::milliseconds timeout) {
    const auto deadline = timeout.count() > 0
                              ? std::chrono::steady_clock::now() + timeout
                              : std::chrono::steady_clock::time_point::max();

    while (autolink::OK()) {
        if (CheckServerReady()) {
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
NodeClient<ActionT>::SendGoal(const Goal& goal,
                              const SendGoalOptions& options) {
    return client_->AsyncSendGoal(goal, options);
}

template <typename ActionT>
std::shared_future<typename NodeClient<ActionT>::WrappedResult>
NodeClient<ActionT>::GetResult(std::shared_ptr<GoalHandle> goal_handle) {
    return client_->AsyncGetResult(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> NodeClient<ActionT>::CancelGoal(
    std::shared_ptr<GoalHandle> goal_handle) {
    return client_->AsyncCancelGoal(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> NodeClient<ActionT>::CancelAllGoals() {
    return client_->AsyncCancelAllGoals();
}

template <typename ActionT>
std::optional<typename NodeClient<ActionT>::WrappedResult>
NodeClient<ActionT>::AwaitGoalResult(
    const Goal& goal, const SendGoalOptions& options,
    const std::chrono::milliseconds accept_timeout,
    const std::chrono::milliseconds result_timeout) {
    const auto accepted_future = SendGoal(goal, options);
    if (accepted_future.wait_for(accept_timeout) != std::future_status::ready) {
        AERROR << action_name_ << ": timeout waiting for goal acceptance.";
        return std::nullopt;
    }

    const auto goal_handle = accepted_future.get();
    if (!goal_handle) {
        AWARN << action_name_ << ": goal rejected by action server.";
        return std::nullopt;
    }

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

template class NodeClient<automsgs::actions::NavigateToPoseAction>;
template class NodeClient<automsgs::actions::NavigateThroughPosesAction>;
template class NodeClient<automsgs::actions::DriveOnHeadingAction>;
template class NodeClient<automsgs::actions::BackUpAction>;
template class NodeClient<automsgs::actions::SpinAction>;

}  // namespace bridge
}  // namespace autonomy
