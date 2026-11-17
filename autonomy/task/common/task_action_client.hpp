/*
 * Copyright 2026 The Openbot Authors
 *
 * Task-scoped autolink action client (cross-process RPC to planning/control).
 */

#pragma once

#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "autolink/action/client.hpp"
#include "autolink/node/node.hpp"

namespace autonomy {
namespace task {
namespace common {

/**
 * @brief Thin wrapper around autolink::action::Client for task-process BT plugins.
 */
template <typename ActionT>
class TaskActionClient
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using AutolinkClient = autolink::action::Client<ActionT>;
    using GoalHandle = autolink::action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;
    using SendGoalOptions = typename AutolinkClient::SendGoalOptions;
    using SharedPtr = std::shared_ptr<TaskActionClient<ActionT>>;

    TaskActionClient(std::shared_ptr<autolink::Node> node,
                     std::string action_name);

    const std::string& action_name() const { return action_name_; }
    bool ActionServerIsReady() const;
    bool WaitForServer(
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100),
        std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

    std::shared_future<std::shared_ptr<GoalHandle>> AsyncSendGoal(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions());
    std::shared_future<WrappedResult> AsyncGetResult(
        std::shared_ptr<GoalHandle> goal_handle);
    std::shared_future<bool> AsyncCancelGoal(
        std::shared_ptr<GoalHandle> goal_handle);
    std::shared_future<bool> AsyncCancelAllGoals();
    // Safe in a delayed goal-response callback: it does not retain the wrapper
    // or dereference it after shutdown, and only cancels the specified UUID.
    std::function<void(std::shared_ptr<GoalHandle>)> MakeGoalCancelCallback() const
    {
        const std::weak_ptr<AutolinkClient> transport = client_;
        return [transport](std::shared_ptr<GoalHandle> goal) {
            if (auto client = transport.lock(); client && goal) {
                try { client->AsyncCancelGoal(std::move(goal)); } catch (...) {}
            }
        };
    }

    // Acceptance and cancellation share one lock. A flag plus future.ready()
    // alone misses cancellation between the response callback and promise.set.
    class GoalCancellation {
    public:
        explicit GoalCancellation(std::function<void(std::shared_ptr<GoalHandle>)> cancel)
            : cancel_(std::move(cancel)) {}
        void OnAccepted(std::shared_ptr<GoalHandle> goal) {
            std::shared_ptr<GoalHandle> to_cancel;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                accepted_ = std::move(goal);
                if (requested_ && accepted_ && !sent_) { sent_ = true; to_cancel = accepted_; }
            }
            if (to_cancel) cancel_(std::move(to_cancel));
        }
        void RequestCancel() {
            std::shared_ptr<GoalHandle> to_cancel;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                requested_ = true;
                if (accepted_ && !sent_) { sent_ = true; to_cancel = accepted_; }
            }
            if (to_cancel) cancel_(std::move(to_cancel));
        }
    private:
        std::mutex mutex_;
        bool requested_{false}, sent_{false};
        std::shared_ptr<GoalHandle> accepted_;
        const std::function<void(std::shared_ptr<GoalHandle>)> cancel_;
    };
    std::shared_ptr<GoalCancellation> MakeGoalCancellation() const {
        return std::make_shared<GoalCancellation>(MakeGoalCancelCallback());
    }

    std::optional<WrappedResult> SendGoalAndWait(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions(),
        std::chrono::milliseconds accept_timeout = std::chrono::seconds(30),
        std::chrono::milliseconds result_timeout = std::chrono::seconds(300));

private:
    std::shared_ptr<autolink::Node> node_;
    std::string action_name_;
    std::shared_ptr<AutolinkClient> client_;
};

}  // namespace common
}  // namespace task
}  // namespace autonomy

#include "autonomy/task/common/task_action_client_impl.hpp"
