/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file action_send.hpp
 * @brief SendActionGoal helper (worker wait for accept).
 *
 * @details
 * Thin wrapper around NodeClient::SendGoal that blocks the calling worker
 * until the action server accepts (or rejects / times out) a goal. Feedback
 * and result callbacks are wired into the caller-supplied builders.
 *
 * @par Ownership
 * Does not own the ClientT; returned GoalHandle is a shared_ptr from the
 * action client. Callers retain handles as needed for Cancel / Pause.
 *
 * @par Threading
 * Must run on a worker / pool thread: `wait_for(accept_timeout)` blocks.
 * Never call from the gRPC event thread.
 *
 * @par Invariants
 * - Must run on a worker / pool thread: wait_for(accept_timeout) blocks.
 * - Returns kServerNotReady without sending when CheckServerReady is false.
 * - on_result_clear (if set) runs before build_result on the result callback.
 *
 * @see ActionBackgroundInterface
 * @see RunAction
 */

#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include "autonomy/bridge/node_client.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Outcome of waiting for action-goal acceptance.
 *
 * @details
 * Distinguishes accept success from timeout, explicit rejection, and a
 * not-ready action server (no SendGoal attempted).
 */
enum class ActionAcceptStatus {
    /** @brief Goal accepted; handle is non-null. */
    kAccepted = 0,
    /** @brief Accept future did not become ready within accept_timeout. */
    kTimeout,
    /** @brief Server returned a null / rejected handle. */
    kRejected,
    /** @brief Client::CheckServerReady() was false; goal was not sent. */
    kServerNotReady,
};

/**
 * @brief Result of SendActionGoal.
 *
 * @tparam GoalHandleT Client goal-handle type.
 *
 * @note handle is non-null only when status == kAccepted.
 */
template <typename GoalHandleT>
struct ActionSendResult {
    /**
     * @brief Shared / weak / unique pointer aliases for ActionSendResult.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ActionSendResult<GoalHandleT>)

    /**
     * @brief Acceptance outcome (accepted / timeout / rejected / not ready).
     */
    ActionAcceptStatus status{ActionAcceptStatus::kRejected};

    /**
     * @brief Accepted goal handle; null unless status == kAccepted.
     *
     * @details Shared ownership with the action client; keep alive for
     * CancelGoal / PauseGoal while the action is running.
     */
    std::shared_ptr<GoalHandleT> handle;
};

/**
 * @brief Send an action goal with feedback/result mapping.
 *
 * @tparam ClientT         `NodeClient<ActionT>`.
 * @tparam BuildFeedbackFn `(const Feedback&) -> void` (usually emits a response).
 * @tparam BuildResultFn   `(const WrappedResult&) -> void`.
 *
 * @param[in,out] client          Action client used for SendGoal.
 * @param[in]     goal            Action goal payload.
 * @param[in]     build_feedback  Invoked on each feedback frame.
 * @param[in]     build_result    Invoked on terminal wrapped result.
 * @param[in]     on_result_clear Optional cleanup before build_result.
 * @param[in]     accept_timeout  Max wait for goal acceptance.
 * @return                        ActionSendResult with status and optional handle.
 *
 * @warning Blocks the calling thread until accept or timeout.
 * @note Feedback / result callbacks may run on action-client threads.
 *
 * @see ActionBackgroundInterface::StartAction
 */
template <typename ClientT, typename BuildFeedbackFn, typename BuildResultFn>
ActionSendResult<typename ClientT::GoalHandle> SendActionGoal(
    ClientT& client, const typename ClientT::Goal& goal,
    BuildFeedbackFn&& build_feedback, BuildResultFn&& build_result,
    std::function<void()> on_result_clear = nullptr,
    std::chrono::milliseconds accept_timeout = std::chrono::seconds(30)) {
    /** @brief GoalHandle type for @p ClientT (accepted-goal handle). */
    using GoalHandle = typename ClientT::GoalHandle;
    ActionSendResult<GoalHandle> result;

    if (!client.CheckServerReady()) {
        result.status = ActionAcceptStatus::kServerNotReady;
        return result;
    }

    typename ClientT::SendGoalOptions options;
    options.feedback_callback =
        [build_feedback = std::forward<BuildFeedbackFn>(build_feedback)](
            std::shared_ptr<GoalHandle>,
            std::shared_ptr<const typename ClientT::Feedback> feedback) mutable {
            if (!feedback) {
                return;
            }
            build_feedback(*feedback);
        };
    options.result_callback =
        [build_result = std::forward<BuildResultFn>(build_result),
         on_result_clear = std::move(on_result_clear)](
            const typename GoalHandle::WrappedResult& wrapped) mutable {
            if (on_result_clear) {
                on_result_clear();
            }
            build_result(wrapped);
        };

    const auto accepted_future = client.SendGoal(goal, options);
    if (accepted_future.wait_for(accept_timeout) != std::future_status::ready) {
        result.status = ActionAcceptStatus::kTimeout;
        return result;
    }
    result.handle = accepted_future.get();
    result.status = result.handle ? ActionAcceptStatus::kAccepted
                            : ActionAcceptStatus::kRejected;
    return result;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
