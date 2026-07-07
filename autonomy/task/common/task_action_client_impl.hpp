/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <thread>

#include "autolink/action/create_client.hpp"
#include "autolink/state.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace task {
namespace common {

template <typename ActionT>
TaskActionClient<ActionT>::TaskActionClient(std::shared_ptr<autolink::Node> node,
                                              std::string action_name)
    : node_(std::move(node)), action_name_(std::move(action_name))
{
    client_ = autolink::action::CreateClient<ActionT>(node_, action_name_);
}

template <typename ActionT>
bool TaskActionClient<ActionT>::ActionServerIsReady() const
{
    return client_ && client_->ActionServerIsReady();
}

template <typename ActionT>
bool TaskActionClient<ActionT>::WaitForServer(
    const std::chrono::milliseconds poll_interval,
    const std::chrono::milliseconds timeout)
{
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
std::shared_future<std::shared_ptr<typename TaskActionClient<ActionT>::GoalHandle>>
TaskActionClient<ActionT>::AsyncSendGoal(const Goal& goal,
                                         const SendGoalOptions& options)
{
    return client_->AsyncSendGoal(goal, options);
}

template <typename ActionT>
std::shared_future<typename TaskActionClient<ActionT>::WrappedResult>
TaskActionClient<ActionT>::AsyncGetResult(std::shared_ptr<GoalHandle> goal_handle)
{
    return client_->AsyncGetResult(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> TaskActionClient<ActionT>::AsyncCancelGoal(
    std::shared_ptr<GoalHandle> goal_handle)
{
    return client_->AsyncCancelGoal(goal_handle);
}

template <typename ActionT>
std::shared_future<bool> TaskActionClient<ActionT>::AsyncCancelAllGoals()
{
    return client_->AsyncCancelAllGoals();
}

template <typename ActionT>
std::optional<typename TaskActionClient<ActionT>::WrappedResult>
TaskActionClient<ActionT>::SendGoalAndWait(
    const Goal& goal, const SendGoalOptions& options,
    const std::chrono::milliseconds accept_timeout,
    const std::chrono::milliseconds result_timeout)
{
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

}  // namespace common
}  // namespace task
}  // namespace autonomy
