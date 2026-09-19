/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file action_background_interface.hpp
 * @brief CRTP ActionBackgroundInterface + RunAction helper.
 *
 * @details
 * Provides the shared Gate → Accept ACK → work-pool Execute path used by
 * navigator and teleop relative Action backends. SendActionGoal runs on the
 * scheduled Execute lambda (not the gRPC event thread).
 *
 * @par Ownership
 * Does not own Client / Session; callers pass references. Hooks are
 * value-captured into the Execute lambda.
 *
 * @par Threading
 * StartAction is typically called from the gRPC thread (non-blocking after
 * schedule). Accept wait and feedback/result callbacks run on the worker /
 * action-client threads.
 *
 * @par Invariants
 * - StartAction validates callback / server readiness / optional sync_reject
 *   before session.Start; reject frames are emitted by the caller path.
 * - SendActionGoal runs on the work-pool Execute path (not the gRPC event
 *   thread); accept wait uses Derived::kAcceptTimeout.
 * - Hooks are value-captured into the Execute lambda; on_accepted runs only
 *   after kAccepted.
 *
 * @see SendActionGoal
 * @see BackgroundCommandSession
 * @see RunAction
 */

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include "autonomy/bridge/grpc/clients/action_send.hpp"
#include "autonomy/bridge/grpc/session.hpp"
#include "autonomy/common/helper_functions/crtp.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Runtime hooks for one Action-backed background start.
 *
 * @tparam Derived Action policy (CRTP leaf).
 *
 * @note All hooks are optional; unset callables are skipped.
 */
template <typename Derived>
struct ActionBackgroundHooks {
    /**
     * @brief Shared / weak / unique pointer aliases for ActionBackgroundHooks.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ActionBackgroundHooks<Derived>)

    /**
     * @brief Goal-handle type from Derived::Client.
     */
    using GoalHandle = typename Derived::Client::GoalHandle;

    /**
     * @brief Stream response type from Derived.
     */
    using Response = typename Derived::Response;

    /**
     * @brief Optional pre-start reject hook.
     *
     * @details Invoked before session.Start. Return a message to reject, or
     * nullopt to continue.
     */
    std::function<std::optional<std::string>()> sync_reject;

    /**
     * @brief Called under no assumed lock after goal accept.
     *
     * @details Receives the GoalHandle shared_ptr from SendActionGoal when
     * status == kAccepted. Typical use: stash the handle for Cancel/Pause.
     */
    std::function<void(std::shared_ptr<GoalHandle>)> on_accepted;

    /**
     * @brief When true, skip emitting feedback frames.
     *
     * @details Polled on each feedback callback; return true to drop the
     * current conversion/emit.
     */
    std::function<bool()> skip_feedback;

    /**
     * @brief Invoked before result emit (e.g. clear muxer / handles).
     *
     * @details Runs on the result callback path before MakeResult.
     */
    std::function<void()> on_result_clear;
};

/**
 * @brief CRTP interface: Gate → Accept ACK → pool Execute via SendActionGoal.
 *
 * @tparam Derived Concrete action policy providing ConvertToGoal, MakeAccept,
 *                MakeReject, MakeFeedback, MakeResult, MakeAcceptFailure, CmdId,
 *                ClientId, ServerNotReadyMessage, and optional kEmitAfterAccept /
 *                kAcceptTimeout.
 *
 * @par Ownership
 * Stateless policy interface; session / client owned by the caller.
 *
 * @par Threading
 * StartAction schedules Execute on BackgroundCommandSession's work path;
 * accept wait blocks that worker, not the gRPC event thread.
 *
 * @warning Derived must be default-constructible when used via RunAction.
 * @see RunAction
 * @see SendActionGoal
 */
template <typename Derived>
class ActionBackgroundInterface
    : public common::helper_functions::crtp<Derived>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for ActionBackgroundInterface.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ActionBackgroundInterface<Derived>)

    /**
     * @brief Action NodeClient type from Derived.
     */
    using Client = typename Derived::Client;

    /**
     * @brief Rpc request type from Derived.
     */
    using Request = typename Derived::Request;

    /**
     * @brief Stream response type from Derived.
     */
    using Response = typename Derived::Response;

    /**
     * @brief Stream sink for accept / feedback / result / reject frames.
     */
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Gate + immediate Accept ACK + schedule Execute.
     *
     * @param[in,out] session  Background command session owning the stream.
     * @param[in,out] client   Action NodeClient used for SendGoal.
     * @param[in]     request  Rpc request converted via Derived::ConvertToGoal.
     * @param[in]     callback Stream sink for accept / feedback / result / reject.
     * @param[in]     hooks    Optional sync_reject / on_accepted / skip_feedback /
     *                        on_result_clear hooks.
     * @return false if rejected before schedule (reject already emitted).
     */
    bool StartAction(BackgroundCommandSession<Response>& session, Client& client,
               const Request& request, StreamCallback callback,
               ActionBackgroundHooks<Derived> hooks) const {
        const Derived& policy = this->impl();

        if (!callback) {
            return false;
        }
        if (!client.CheckServerReady()) {
            callback(policy.MakeReject(
                request, policy.ServerNotReadyMessage()));
            return false;
        }
        if (hooks.sync_reject) {
            if (const auto message = hooks.sync_reject()) {
                callback(policy.MakeReject(request, *message));
                return false;
            }
        }

        return session.Start(
            std::move(callback), policy.MakeAccept(request),
            [policy, request](const std::string& message) {
                return policy.MakeReject(request, message);
            },
            [&client, policy, request, hooks = std::move(hooks)](
                const StreamCallback& emit) mutable {
                auto result = SendActionGoal(
                    client, policy.ConvertToGoal(request),
                    [policy, request, hooks, emit](
                        const typename Client::Feedback& feedback) {
                        if (hooks.skip_feedback && hooks.skip_feedback()) {
                            return;
                        }
                        emit(policy.MakeFeedback(request, feedback));
                    },
                    [policy, request, hooks, emit](const auto& wrapped) {
                        if (hooks.on_result_clear) {
                            hooks.on_result_clear();
                        }
                        emit(policy.MakeResult(request, wrapped));
                    },
                    nullptr, Derived::kAcceptTimeout);

                if (result.status != ActionAcceptStatus::kAccepted) {
                    emit(policy.MakeAcceptFailure(request, result.status));
                    return false;
                }
                if (hooks.on_accepted) {
                    hooks.on_accepted(result.handle);
                }
                if constexpr (Derived::kEmitAfterAccept) {
                    emit(policy.MakeAfterAccept(request));
                }
                return true;
            },
            policy.CmdId(request), policy.ClientId(request));
    }
};

/**
 * @brief Convenience: construct Derived and call ActionBackgroundInterface::StartAction.
 *
 * @tparam Derived  Action policy CRTP leaf.
 * @param[in,out] session  Background command session.
 * @param[in,out] client   Action client.
 * @param[in]     request  Rpc request.
 * @param[in]     callback Stream sink.
 * @param[in]     hooks    Optional hooks pack.
 * @return                 Same as Derived::StartAction.
 *
 * @note Requires Derived to be default-constructible.
 */
template <typename Derived>
bool RunAction(
    BackgroundCommandSession<typename Derived::Response>& session,
    typename Derived::Client& client, const typename Derived::Request& request,
    std::function<void(const typename Derived::Response&)> callback,
    ActionBackgroundHooks<Derived> hooks) {
    return Derived{}.StartAction(session, client, request, std::move(callback),
                           std::move(hooks));
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
