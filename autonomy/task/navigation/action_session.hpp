/*
 * Copyright 2026 The Openbot Authors
 *
 * Polls a cross-process autolink action goal from BT stateful nodes.
 */

#pragma once

#include <functional>
#include <future>
#include <memory>

#include <behaviortree_cpp/bt_factory.h>

#include "autolink/action/client.hpp"
#include "autonomy/task/common/task_action_client.hpp"

namespace autonomy {
namespace task {
namespace navigation {

template <typename ActionT>
class ActionSession
{
public:
    using Client = common::TaskActionClient<ActionT>;
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = typename Client::GoalHandle;
    using WrappedResult = typename Client::WrappedResult;

    void Begin(Client& client, const Goal& goal);
    void Cancel(Client& client);
    /** Drop local state without contacting the server (completed / stale). */
    void Reset();

    template <typename OnSuccess, typename OnFailure>
    BT::NodeStatus Tick(const std::function<bool()>& cancel_checker,
                        OnSuccess on_success, OnFailure on_failure);

    bool has_feedback() const { return has_feedback_; }
    const Feedback& latest_feedback() const { return latest_feedback_; }
    /** True if an accepted goal handle is held (CancelAll is meaningful). */
    bool has_goal_handle() const { return static_cast<bool>(goal_handle_); }
    /** True while accepting or executing a remote goal. */
    bool is_busy() const
    {
        return phase_ == Phase::kAccepting || phase_ == Phase::kRunning;
    }

private:
    enum class Phase { kIdle, kAccepting, kRunning };

    std::shared_future<std::shared_ptr<GoalHandle>> accept_future_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_future<WrappedResult> result_future_;
    Feedback latest_feedback_;
    bool has_feedback_{false};
    Phase phase_{Phase::kIdle};
};

template <typename ActionT>
void ActionSession<ActionT>::Begin(Client& client, const Goal& goal)
{
    // Drop any prior in-flight accept/result before sending a new goal.
    Cancel(client);

    typename Client::SendGoalOptions options;
    options.feedback_callback =
        [this](std::shared_ptr<GoalHandle>,
               const std::shared_ptr<const Feedback> feedback) {
            if (feedback) {
                latest_feedback_ = *feedback;
                has_feedback_ = true;
            }
        };
    accept_future_ = client.AsyncSendGoal(goal, options);
    goal_handle_.reset();
    result_future_ = {};
    has_feedback_ = false;
    phase_ = Phase::kAccepting;
}

template <typename ActionT>
void ActionSession<ActionT>::Cancel(Client& client)
{
    // Only cancel a known accepted goal that is still in flight. Canceling a
    // completed goal (or CancelAll after success) can hang get_result peers
    // and leave the next 2D Goal with no FollowPath accept.
    if (goal_handle_ && is_busy()) {
        try {
            client.AsyncCancelGoal(goal_handle_);
        } catch (const std::exception&) {
        }
    }
    Reset();
}

template <typename ActionT>
void ActionSession<ActionT>::Reset()
{
    phase_ = Phase::kIdle;
    goal_handle_.reset();
    accept_future_ = {};
    result_future_ = {};
    has_feedback_ = false;
}

template <typename ActionT>
template <typename OnSuccess, typename OnFailure>
BT::NodeStatus ActionSession<ActionT>::Tick(
    const std::function<bool()>& cancel_checker, OnSuccess on_success,
    OnFailure on_failure)
{
    if (cancel_checker && cancel_checker()) {
        return BT::NodeStatus::FAILURE;
    }

    try {
        if (phase_ == Phase::kAccepting) {
            if (accept_future_.wait_for(std::chrono::seconds(0)) !=
                std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            goal_handle_ = accept_future_.get();
            if (!goal_handle_) {
                on_failure(-1, "action goal rejected");
                Reset();
                return BT::NodeStatus::FAILURE;
            }
            result_future_ = goal_handle_->AsyncGetResult();
            phase_ = Phase::kRunning;
        }

        if (phase_ == Phase::kRunning) {
            if (result_future_.wait_for(std::chrono::seconds(0)) !=
                std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            const WrappedResult wrapped = result_future_.get();
            const bool ok =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED &&
                wrapped.result;
            if (ok) {
                on_success(*wrapped.result);
                Reset();
                return BT::NodeStatus::SUCCESS;
            }
            on_failure(static_cast<int>(wrapped.code), "action failed");
            Reset();
            return BT::NodeStatus::FAILURE;
        }
    } catch (const std::exception& ex) {
        Reset();
        on_failure(-1, ex.what());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
