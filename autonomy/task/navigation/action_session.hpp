/*
 * Copyright 2026 The Openbot Authors
 *
 * Polls a cross-process autolink action goal from BT stateful nodes.
 */

#pragma once

#include <functional>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>

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

    bool TryGetFeedback(Feedback* feedback, double max_age = 1.0) const;
    bool has_feedback() const { Feedback feedback; return TryGetFeedback(&feedback); }
    Feedback latest_feedback() const { Feedback feedback; TryGetFeedback(&feedback); return feedback; }
    /** True if an accepted goal handle is held (CancelAll is meaningful). */
    bool has_goal_handle() const
    {
        std::lock_guard<std::recursive_mutex> lock(session_mutex_);
        return static_cast<bool>(goal_handle_);
    }
    /** True while accepting or executing a remote goal. */
    bool is_busy() const
    {
        std::lock_guard<std::recursive_mutex> lock(session_mutex_);
        return phase_ == Phase::kAccepting || phase_ == Phase::kRunning;
    }

private:
    enum class Phase { kIdle, kAccepting, kRunning };

    std::shared_future<std::shared_ptr<GoalHandle>> accept_future_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_future<WrappedResult> result_future_;
    struct FeedbackState {
        std::mutex mutex;
        Feedback latest;
        bool valid{false};
        std::chrono::steady_clock::time_point received{};
    };
    mutable std::recursive_mutex session_mutex_;
    std::shared_ptr<FeedbackState> feedback_state_;
    std::shared_ptr<typename Client::GoalCancellation> cancellation_;
    Phase phase_{Phase::kIdle};
};

template <typename ActionT>
void ActionSession<ActionT>::Begin(Client& client, const Goal& goal)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    const auto previous_cancellation = cancellation_;
    // If a goal is already executing, send the replacement without Cancel.
    // Autolink SimpleActionServer treats a second send_goal as preempt and
    // switches to the pending goal (keeps FollowPath motion continuous across
    // RateController replans). Cancel+resend zeros cmd_vel and spams
    // "Client requested to cancel".
    if (phase_ == Phase::kAccepting) {
        if (cancellation_) cancellation_->RequestCancel();
        // Previous accept still in flight — drop local wait; server will see
        // the newer send_goal as preempt/replace.
        goal_handle_.reset();
        accept_future_ = {};
        result_future_ = {};
    } else if (phase_ == Phase::kRunning) {
        goal_handle_.reset();
        result_future_ = {};
    }

    typename Client::SendGoalOptions options;
    feedback_state_ = std::make_shared<FeedbackState>();
    cancellation_ = client.MakeGoalCancellation();
    const auto cancellation = cancellation_;
    options.goal_response_callback = [cancellation](std::shared_ptr<GoalHandle> goal) {
        cancellation->OnAccepted(std::move(goal));
    };
    // Neither a late previous-goal callback nor a callback after destruction
    // may mutate the current session. The callback never captures this.
    const std::weak_ptr<FeedbackState> feedback_state = feedback_state_;
    options.feedback_callback =
        [feedback_state](std::shared_ptr<GoalHandle>,
               const std::shared_ptr<const Feedback> feedback) {
            if (auto state = feedback_state.lock(); state && feedback) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->latest = *feedback;
                state->received = std::chrono::steady_clock::now();
                state->valid = true;
            }
        };
    phase_ = Phase::kAccepting;
    try {
        accept_future_ = client.AsyncSendGoal(goal, options);
    } catch (...) {
        if (previous_cancellation) previous_cancellation->RequestCancel();
        if (cancellation_) cancellation_->RequestCancel();
        Reset();
        throw;
    }
}

template <typename ActionT>
void ActionSession<ActionT>::Cancel(Client& /*client*/)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    if (is_busy() && cancellation_) cancellation_->RequestCancel();
    Reset();
}

template <typename ActionT>
void ActionSession<ActionT>::Reset()
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    phase_ = Phase::kIdle;
    goal_handle_.reset();
    accept_future_ = {};
    result_future_ = {};
    feedback_state_.reset();
    cancellation_.reset();
}

template <typename ActionT>
bool ActionSession<ActionT>::TryGetFeedback(Feedback* feedback, double max_age) const
{
    if (!feedback || !std::isfinite(max_age) || max_age < 0.0) return false;
    std::shared_ptr<FeedbackState> state;
    {
        std::lock_guard<std::recursive_mutex> lock(session_mutex_);
        state = feedback_state_;
    }
    if (!state) return false;
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->valid || std::chrono::duration<double>(
            std::chrono::steady_clock::now() - state->received).count() > max_age) return false;
    *feedback = state->latest;
    return true;
}

template <typename ActionT>
template <typename OnSuccess, typename OnFailure>
BT::NodeStatus ActionSession<ActionT>::Tick(
    const std::function<bool()>& cancel_checker, OnSuccess on_success,
    OnFailure on_failure)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    if (cancel_checker && cancel_checker()) {
        if (cancellation_) cancellation_->RequestCancel();
        Reset();
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
                wrapped.result && wrapped.result->error_code() == 0;
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
        if (cancellation_) cancellation_->RequestCancel();
        Reset();
        on_failure(-1, ex.what());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
