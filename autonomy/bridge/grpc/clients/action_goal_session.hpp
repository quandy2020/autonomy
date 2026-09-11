/*
 * Copyright 2026 The Openbot Authors
 *
 * Template helper that sends an action goal and wires feedback/result.
 */

#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include "autonomy/bridge/node_client.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/** @brief Outcome of waiting for action-goal acceptance. */
enum class ActionAcceptStatus {
    kAccepted = 0,
    kTimeout,
    kRejected,
    kServerNotReady,
};

/**
 * @brief Result of @ref SendActionGoal.
 * @tparam GoalHandleT Client goal-handle type.
 */
template <typename GoalHandleT>
struct ActionSendResult {
    ActionAcceptStatus status{ActionAcceptStatus::kRejected};
    std::shared_ptr<GoalHandleT> handle;
};

/**
 * @brief Send an action goal with feedback/result mapping.
 *
 * @tparam ClientT `NodeClient<ActionT>`.
 * @tparam BuildFeedbackFn `(const Feedback&) -> void` (usually emits a response).
 * @tparam BuildResultFn `(const WrappedResult&) -> void`.
 */
template <typename ClientT, typename BuildFeedbackFn, typename BuildResultFn>
ActionSendResult<typename ClientT::GoalHandle> SendActionGoal(
    ClientT& client, const typename ClientT::Goal& goal,
    BuildFeedbackFn&& build_feedback, BuildResultFn&& build_result,
    std::function<void()> on_result_clear = nullptr,
    std::chrono::milliseconds accept_timeout = std::chrono::seconds(30)) {
    using GoalHandle = typename ClientT::GoalHandle;
    ActionSendResult<GoalHandle> result;

    if (!client.ActionServerIsReady()) {
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

    const auto accepted_future = client.AsyncSendGoal(goal, options);
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
