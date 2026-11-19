/*
 * Copyright 2026 The Openbot Authors
 *
 * @file action_session.hpp
 * @brief Poll native action goals with revocable, per-goal dispatch ownership.
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
#include "autonomy/task/common/motion_dispatch_gate.hpp"

namespace autonomy {
namespace task {
namespace navigation {

template <typename ActionT, typename ClientT = common::TaskActionClient<ActionT>>
class ActionSession
{
public:
    using Client = ClientT;
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = typename Client::GoalHandle;
    using WrappedResult = typename Client::WrappedResult;

    /** @brief Register ownership and commit one send; false when admission is revoked. */
    bool Begin(Client& client, const Goal& goal,
               const std::shared_ptr<common::MotionDispatchGate>& gate = {});
    void Cancel(Client& client);
    /** Drop local state without contacting the server (completed / stale). */
    void Reset();

    template <typename OnSuccess, typename OnFailure>
    BT::NodeStatus Tick(const std::function<bool()>& cancel_checker,
                        OnSuccess on_success, OnFailure on_failure);

    bool TryGetFeedback(Feedback* feedback, double max_age = 1.0) const;
    /**
     * @brief Check whether retained work needs a fresh send after owner resume.
     * @param gate The currently installed invocation gate, not a new task gate.
     * @return True only after this send has authoritative terminal evidence and
     * the same owner has enabled a newer revision. No send is committed here.
     * Relative recovery actions must preserve their remaining execution intent
     * before Begin() is called again; this query does not authorize repetition.
     */
    bool NeedsResume(const std::shared_ptr<common::MotionDispatchGate>& gate) const;
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
    std::shared_ptr<common::MotionDispatchGate::Ticket> ticket_; ///< This send revision.
    std::weak_ptr<common::MotionDispatchGate> gate_; ///< Identifies rolling replacements.
    Phase phase_{Phase::kIdle}; ///< Polled only under session_mutex_.
};

template <typename ActionT, typename ClientT>
bool ActionSession<ActionT, ClientT>::Begin(
    Client& client, const Goal& goal, const std::shared_ptr<common::MotionDispatchGate>& gate)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    const auto previous = cancellation_;
    const auto feedback_state = std::make_shared<FeedbackState>();
    const auto cancellation = client.MakeGoalCancellation();
    typename Client::SendGoalOptions options;
    cancellation->Configure(options);
    const std::weak_ptr<FeedbackState> feedback = feedback_state;
    options.feedback_callback = [feedback](std::shared_ptr<GoalHandle>,
                                           const std::shared_ptr<const Feedback> sample) {
        if (auto state = feedback.lock(); state && sample) {
            std::lock_guard<std::mutex> guard(state->mutex);
            state->latest = *sample;
            state->received = std::chrono::steady_clock::now();
            state->valid = true;
        }
    };
    std::shared_ptr<common::MotionDispatchGate::Ticket> ticket;
    if (gate) {
        ticket = gate->Reserve([cancellation] { cancellation->RequestCancel(); },
                               [cancellation] { return cancellation->HasFailure(); });
        if (!ticket) return false;
        try { cancellation->SetTerminalCallback([ticket] { ticket->Finish(); }); }
        catch (...) { ticket->Finish(); throw; }
        if (!ticket->Start()) return false;
    }
    // Failed admission must not discard the preceding retained action. Running
    // FollowPath replacements share one invocation and use native preemption;
    // all preceding sends retain their gate-owned cancellation responsibility.
    if (previous && (phase_ == Phase::kAccepting || gate_.lock() != gate))
        previous->RequestCancel();
    Reset();
    feedback_state_ = feedback_state;
    cancellation_ = cancellation;
    ticket_ = std::move(ticket);
    gate_ = gate;
    phase_ = Phase::kAccepting;
    try {
        accept_future_ = client.AsyncSendGoal(goal, options);
        if (!accept_future_.valid()) throw std::runtime_error("invalid action acceptance future");
    } catch (...) {
        cancellation->OnSendFailure();
        cancellation->RequestCancel();
        if (ticket_) ticket_->Fail();
        if (previous) previous->RequestCancel();
        Reset();
        throw;
    }
    return true;
}

template <typename ActionT, typename ClientT>
void ActionSession<ActionT, ClientT>::Cancel(Client& /*client*/)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    if (is_busy() && cancellation_) cancellation_->RequestCancel();
    Reset();
}

template <typename ActionT, typename ClientT>
void ActionSession<ActionT, ClientT>::Reset()
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    phase_ = Phase::kIdle;
    goal_handle_.reset();
    accept_future_ = {};
    result_future_ = {};
    feedback_state_.reset();
    cancellation_.reset();
    ticket_.reset();
    gate_.reset();
}

template <typename ActionT, typename ClientT>
bool ActionSession<ActionT, ClientT>::TryGetFeedback(Feedback* feedback, double max_age) const
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

template <typename ActionT, typename ClientT>
bool ActionSession<ActionT, ClientT>::NeedsResume(
    const std::shared_ptr<common::MotionDispatchGate>& gate) const
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    return phase_ != Phase::kIdle && gate && gate_.lock() == gate && ticket_ &&
           cancellation_ && cancellation_->Terminal() &&
           ticket_->HasResumedRevision();
}

template <typename ActionT, typename ClientT>
template <typename OnSuccess, typename OnFailure>
BT::NodeStatus ActionSession<ActionT, ClientT>::Tick(
    const std::function<bool()>& cancel_checker, OnSuccess on_success,
    OnFailure on_failure)
{
    std::lock_guard<std::recursive_mutex> lock(session_mutex_);
    if (cancel_checker && cancel_checker()) {
        if (cancellation_) cancellation_->RequestCancel();
        Reset();
        return BT::NodeStatus::FAILURE;
    }
    if (ticket_ && ticket_->Revoked()) {
        if (cancellation_) cancellation_->RequestCancel();
        // Pause may arrive after the runner's pre-tick check. Keep the action
        // state so a confirmed terminal and newer owner revision can resume it.
        if (ticket_->Suspended()) return BT::NodeStatus::RUNNING;
        Reset();
        return BT::NodeStatus::FAILURE;
    }

    try {
        if (cancellation_ && cancellation_->HasFailure())
            throw std::runtime_error("native action acceptance/cancellation is uncertain");
        if (phase_ == Phase::kAccepting) {
            if (!accept_future_.valid()) throw std::runtime_error("invalid action acceptance future");
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
            cancellation_->OnAccepted(goal_handle_);
            result_future_ = goal_handle_->AsyncGetResult();
            phase_ = Phase::kRunning;
        }

        if (phase_ == Phase::kRunning) {
            if (!result_future_.valid()) throw std::runtime_error("invalid action result future");
            if (result_future_.wait_for(std::chrono::seconds(0)) !=
                std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            const WrappedResult wrapped = result_future_.get();
            cancellation_->OnTerminal(wrapped);
            if (!cancellation_->Terminal()) throw std::runtime_error("uncorrelated action terminal result");
            if (ticket_ && ticket_->Revoked()) {
                if (ticket_->Suspended()) return BT::NodeStatus::RUNNING;
                Reset();
                return BT::NodeStatus::FAILURE;
            }
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
        if (cancellation_) { cancellation_->OnSendFailure(); cancellation_->RequestCancel(); }
        if (ticket_) ticket_->Fail();
        Reset();
        on_failure(-1, ex.what());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
