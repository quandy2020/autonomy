/**
 * Copyright 2026 The Openbot Authors
 *
 * @file task_action_client.hpp
 * @brief Task-scoped native action client and per-goal cancellation evidence.
 */

#pragma once

#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>

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
    /** @brief Goal-specific cancellation RPC with an observable acceptance future. */
    using GoalCancel = std::function<std::shared_future<bool>(const autolink::action::GoalUUID&)>;

    /**
     * @brief Obtain a callback safe after wrapper shutdown and for uncertain acceptance.
     * @return UUID-targeted cancellation retaining the native service runtime.
     * The callback outlives wrapper/facade release and retains no task or BT node.
     */
    GoalCancel MakeGoalCancelCallback() const {
        if (!client_) throw std::runtime_error("action transport unavailable during cancellation");
        return client_->MakeGoalCancelCallback();
    }

    /**
     * @brief Per-goal acceptance, cancellation and terminal evidence ownership.
     *
     * Callbacks retain this state, never the BT node or session. The state keeps
     * no strong goal-handle reference, preventing a handle/result-callback cycle.
     * Cancel acceptance is observed separately from authoritative termination.
     */
    class GoalCancellation : public std::enable_shared_from_this<GoalCancellation> {
    public:
        /// Construct with one UUID-targeted transport operation.
        explicit GoalCancellation(GoalCancel cancel) : cancel_(std::move(cancel)) {}

        /** @brief Install identity, explicit rejection, error and result callbacks. */
        void Configure(SendGoalOptions& options) {
            const auto state = this->shared_from_this();
            options.goal_request_callback = [state](const auto& id) { state->OnGoalId(id); };
            options.goal_rejection_callback = [state](const auto& id) { state->OnRejected(id); };
            options.goal_error_callback = [state](const auto&, std::exception_ptr) {
                state->OnSendFailure();
            };
            options.goal_response_callback = [state](std::shared_ptr<GoalHandle> goal) {
                state->OnAccepted(std::move(goal));
            };
            options.result_callback = [state](const WrappedResult& result) { state->OnTerminal(result); };
        }
        /** @brief Bind gate settlement before the registered send commitment. */
        void SetTerminalCallback(std::function<void()> callback) {
            const auto notification =
                std::make_shared<const std::function<void()>>(std::move(callback));
            bool terminal;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                terminal_callback_ = notification;
                terminal = terminal_;
            }
            if (terminal) Notify(notification);
        }
        /** @brief Register the UUID before transport; never issue premature cancel. */
        void OnGoalId(const autolink::action::GoalUUID& id) noexcept {
            std::lock_guard<std::mutex> lock(mutex_);
            if (id_ && *id_ != id) { failed_ = true; return; }
            id_ = id;
        }
        /** @brief Bind acceptance/result awareness; late acceptance honors cancel. */
        void OnAccepted(std::shared_ptr<GoalHandle> goal) noexcept {
            if (!goal) return;  // Null alone does not distinguish rejection and timeout.
            try {
                const auto id = goal->GetGoalId();
                const auto result = goal->AsyncGetResult();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (id_ && *id_ != id) { failed_ = true; return; }
                    id_ = id;
                    if (terminal_) return;
                    accepted_ = true;
                    result_future_ = result;
                }
                TryCancel();
            } catch (...) { OnSendFailure(); }
        }
        /** @brief Settle only a real, matching negative send_goal response. */
        void OnRejected(const autolink::action::GoalUUID& id) noexcept {
            std::shared_ptr<const std::function<void()>> callback;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!id_ || *id_ != id) { failed_ = true; return; }
                if (terminal_) return;
                terminal_ = true;
                callback = terminal_callback_;
            }
            Notify(callback);
        }
        /** @brief Confirm a matching terminal result, including result-before-accept. */
        void OnTerminal(const WrappedResult& result) noexcept {
            const bool terminal = result.code == autolink::action::ResultCode::SUCCEEDED ||
                                  result.code == autolink::action::ResultCode::CANCELED ||
                                  result.code == autolink::action::ResultCode::ABORTED;
            std::shared_ptr<const std::function<void()>> callback;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!terminal || !id_ || *id_ != result.goal_id) { failed_ = true; return; }
                if (terminal_) return;
                terminal_ = true;
                callback = terminal_callback_;
            }
            Notify(callback);
        }
        /** @brief Record ambiguous send failure; retain identity for targeted cancel. */
        void OnSendFailure() noexcept {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                failed_ = true;
                send_failed_ = true;
            }
            TryCancel();
        }
        /** @brief Request cancel now, or after a later matching acceptance. */
        void RequestCancel() noexcept {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                requested_ = true;
            }
            TryCancel();
        }
        /**
         * @brief Poll RPC/result uncertainty without retaining a transport lock.
         * @return False after authoritative termination; true on a visible failure.
         * A missing cancel ACK stays pending and is bounded by the owner's drain.
         */
        bool HasFailure() noexcept {
            std::shared_future<bool> cancel;
            std::shared_future<WrappedResult> result;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (terminal_) return false;
                cancel = cancel_future_;
                result = result_future_;
            }
            try {
                if (result.valid() && result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
                    OnTerminal(result.get());
                if (cancel.valid() && cancel.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
                    !cancel.get()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!terminal_) failed_ = true;
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!terminal_) failed_ = true;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            return !terminal_ && failed_;
        }
        /** @return True only for rejection or a matching authoritative terminal result. */
        bool Terminal() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return terminal_;
        }

    private:
        /** @brief Send one cancel attempt outside all state locks. */
        void TryCancel() noexcept {
            std::optional<autolink::action::GoalUUID> id;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (terminal_ || !requested_ || !id_ || attempt_started_ ||
                    (!accepted_ && !send_failed_)) return;
                attempt_started_ = true;
                id = id_;
            }
            try {
                auto future = cancel_(*id);
                std::lock_guard<std::mutex> lock(mutex_);
                if (!future.valid() && !terminal_) failed_ = true;
                cancel_future_ = std::move(future);
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!terminal_) failed_ = true;
                // Responsibility and identity are retained; another explicit
                // cancel can retry, while the owner continues to latch failure.
                attempt_started_ = false;
            }
        }
        /** @brief Terminal callbacks cannot throw into a middleware worker. */
        static void Notify(
            const std::shared_ptr<const std::function<void()>>& callback) noexcept {
            try { if (callback && *callback) (*callback)(); } catch (...) {}
        }
        mutable std::mutex mutex_;             ///< Protects this one goal's evidence.
        std::optional<autolink::action::GoalUUID> id_; ///< Generated transport identity.
        bool requested_{false};               ///< Cancellation requested before/after accept.
        bool accepted_{false};                ///< Matching positive response was observed.
        bool send_failed_{false};             ///< Send uncertainty permits cancel by UUID.
        bool attempt_started_{false};         ///< An attempt is in flight or acknowledged.
        bool failed_{false};                  ///< Visible RPC/transport error, not termination.
        bool terminal_{false};                ///< Real rejection or terminal result evidence.
        std::shared_future<bool> cancel_future_; ///< Cancel RPC acceptance channel.
        std::shared_future<WrappedResult> result_future_; ///< Native terminal/exception channel.
        // Shared notification copies cannot allocate inside noexcept terminal callbacks.
        std::shared_ptr<const std::function<void()>> terminal_callback_; ///< Gate settlement.
        const GoalCancel cancel_;              ///< UUID-targeted callback with weak transport.
    };
    /** @brief Allocate independent ownership for each send, including replacements. */
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
