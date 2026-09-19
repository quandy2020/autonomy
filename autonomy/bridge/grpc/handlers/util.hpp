/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file util.hpp
 * @brief Shared RpcHandler helpers: Context lookup, unary reply, stream relay,
 *        and domain terminal-state predicates.
 *
 * @details
 * Used by every `rpc_*_handlers.hpp` path and by the BRIDGE_* templates in
 * handler_templates.hpp. Helpers never own domain stubs or TaskMuxer state;
 * they only drive async_grpc Send / Finish and optionally log ingress.
 *
 * Call graph (typical stream):
 * @code
 * OnRequest → RelayStream → RequireContext
 *          → stub.Handle*(request, StreamUntil/is_final sink)
 *          → Send(frame) … Finish(OK) when Is*Terminal(frame)
 * @endcode
 *
 * Call graph (typical unary):
 * @code
 * OnRequest → ReplyUnaryWithContext / ReplyStatusWithContext
 *          → build(Context*) → ReplyUnary / ReplyStatus → Finish(OK)
 * @endcode
 *
 * @par Invariants
 * - Missing Context ⇒ ErrorStatus(UNKNOWN,"no context") for Status-bearing
 *   responses, or Finish(INTERNAL,"no context") via RequireContext; never
 *   dereference a null Context*.
 * - RelayStream finishes when @p is_final returns true; does not own Stub
 *   session state (stubs ClearSession / Release themselves).
 * - LogIngress is best-effort and must not throw.
 * - Terminal predicates are pure functions of the response frame; they do not
 *   consult Context or stubs.
 *
 * @par Threading
 * Invoked on the async_grpc completion-queue / handler thread. Stream sinks
 * built by RelayStream / StreamUntil may be invoked from Autolink reader or
 * WorkScheduler threads when stubs relay feedback — Send / Finish must remain
 * safe as required by async_grpc.
 *
 * @par Ownership
 * No ownership of Context, stubs, or response lifetimes beyond the Send()
 * unique_ptr hand-off. Handlers remain the sole owners of the RPC.
 *
 * @see handler_templates.hpp
 * @see Context
 * @see rpc_status.hpp
 */

#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "autonomy/bridge/grpc/context.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/common/macros.hpp"
#include "autolink/common/log.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/charge.pb.h>
#include <automsgs/rpcs/exploration.pb.h>
#include <automsgs/rpcs/follow.pb.h>
#include <automsgs/rpcs/navigation.pb.h>
#include <automsgs/rpcs/teleop.pb.h>
#include <automsgs/rpcs/voice.pb.h>
#include <grpc++/grpc++.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {
namespace detail {

/**
 * @brief Primary trait: @p T has no `mutable_status()` member.
 *
 * @details
 * Used by ReplyUnaryWithContext to decide whether a missing Context can still
 * produce an ErrorStatus-bearing response body. Defaults to false; the
 * partial specialization below detects protobuf responses that embed
 * `common.Status` via `mutable_status()`.
 *
 * @tparam T Candidate response type.
 *
 * @note This is a compile-time type trait, not a runtime object.
 */
template <typename T, typename = void>
struct HasMutableStatus : std::false_type {
    /**
     * @brief Shared / weak / unique pointer aliases (API uniformity).
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(HasMutableStatus<T>)
};

/**
 * @brief Specialization: @p T exposes `mutable_status()`.
 *
 * @details
 * Detected via SFINAE on `std::declval<T&>().mutable_status()`. When true,
 * ReplyUnaryWithContext can fill UNKNOWN/"no context" into the response
 * instead of sending an empty default-constructed body.
 *
 * @tparam T Response type with a mutable_status() accessor returning a
 *           pointer to a Status-like field.
 */
template <typename T>
struct HasMutableStatus<
    T, std::void_t<decltype(std::declval<T&>().mutable_status())>>
    : std::true_type {
    /**
     * @brief Shared / weak / unique pointer aliases (API uniformity).
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(HasMutableStatus<T>)
};

}  // namespace detail

/**
 * @brief Log `goal_id` at command ingress when the request carries one.
 *
 * @details
 * Best-effort observability for START-like RPCs. Empty `goal_id()` is a
 * no-op so unary probes without ids stay quiet. Never throws.
 *
 * @tparam RequestT Request type exposing `goal_id()` (e.g. NavigateRequest,
 *                  GoalRequest).
 * @param[in] where   Method / handler label printed before the id
 *                    (typically `Signature::MethodName()`).
 * @param[in] request Incoming request; empty goal_id skips logging.
 *
 * @note Safe to call before RequireContext / RelayStream.
 */
template <typename RequestT>
void LogIngress(const char* where, const RequestT& request) {
    if (request.goal_id().empty()) {
        return;
    }
    AINFO << where << " goal_id=" << request.goal_id();
}

/**
 * @brief Fetch Context from the handler or Finish(INTERNAL, "no context").
 *
 * @details
 * Uses async_grpc `GetUnsynchronizedContext<Context>()`. On null Context the
 * RPC is finished with gRPC INTERNAL (not an application Status body) and
 * nullptr is returned so callers can early-return without dereferencing.
 *
 * Prefer ReplyUnaryWithContext / ReplyStatusWithContext for unary paths that
 * can still emit an application-level ErrorStatus. Prefer this helper when
 * the handler must abort before opening a stream (RelayStream).
 *
 * @tparam HandlerT async_grpc RpcHandler specialization.
 * @param[in,out] handler Handler owning the RPC; may Finish on failure.
 * @return                Non-null Context* on success; nullptr after Finish.
 *
 * @warning After a nullptr return the RPC is already finished — do not Send.
 */
template <typename HandlerT>
Context* RequireContext(HandlerT* handler) {
    auto* context = handler->template GetUnsynchronizedContext<Context>();
    if (!context) {
        handler->Finish(::grpc::Status(::grpc::StatusCode::INTERNAL,
                                       "no context"));
    }
    return context;
}

/**
 * @brief Send a heap-allocated unary response and Finish OK.
 *
 * @details
 * Transfers ownership of @p response into async_grpc via Send, then finishes
 * the RPC with `::grpc::Status::OK`. Application-level success/failure is
 * expected to live inside the protobuf (e.g. `status` field), not the gRPC
 * status code.
 *
 * @tparam HandlerT  async_grpc RpcHandler specialization.
 * @tparam ResponseT Response protobuf type.
 * @param[in,out] handler  Handler owning Send / Finish.
 * @param[in]     response Unique ownership transferred to Send (must be
 *                         non-null).
 */
template <typename HandlerT, typename ResponseT>
void ReplyUnary(HandlerT* handler, std::unique_ptr<ResponseT> response) {
    handler->Send(std::move(response));
    handler->Finish(::grpc::Status::OK);
}

/**
 * @brief Send a by-value unary response and Finish OK.
 *
 * @details
 * Convenience overload that heap-allocates @p response and forwards to the
 * unique_ptr overload.
 *
 * @tparam HandlerT  async_grpc RpcHandler specialization.
 * @tparam ResponseT Response protobuf type.
 * @param[in,out] handler  Handler owning Send / Finish.
 * @param[in]     response Response moved into a unique_ptr for Send.
 */
template <typename HandlerT, typename ResponseT>
void ReplyUnary(HandlerT* handler, ResponseT response) {
    ReplyUnary(handler, std::make_unique<ResponseT>(std::move(response)));
}

/**
 * @brief Reply with a `common.Status` unary body and Finish OK.
 *
 * @details
 * Thin wrapper around ReplyUnary for RPCs whose response type is exactly
 * `::automsgs::rpcs::common::Status` (Cancel / Pause / Estop / etc.).
 *
 * @tparam HandlerT async_grpc RpcHandler specialization.
 * @param[in,out] handler Handler owning Send / Finish.
 * @param[in]     status  Application-level Status moved into the reply.
 */
template <typename HandlerT>
void ReplyStatus(HandlerT* handler,
                 ::automsgs::rpcs::common::Status status) {
    ReplyUnary(handler, std::move(status));
}

/**
 * @brief Build a unary response with Context, or ErrorStatus if Context is
 *        missing.
 *
 * @details
 * Looks up Context without finishing the RPC on failure (unlike
 * RequireContext). When Context is null:
 * - if ResponseT has `mutable_status()`, fills UNKNOWN/"no context";
 * - otherwise sends a default-constructed ResponseT.
 * When Context is live, invokes @p build(context) and ReplyUnary the result.
 *
 * Used by BRIDGE_UNARY / BRIDGE_GET / BRIDGE_BUILD / BRIDGE_WRAP templates.
 *
 * @tparam HandlerT async_grpc RpcHandler specialization.
 * @tparam BuildFn  Callable `(Context*) → ResponseT`.
 * @param[in,out] handler Handler owning the reply path.
 * @param[in]     build   Response factory invoked with a live Context*; must
 *                        not retain the pointer past the call.
 *
 * @note @p build is not invoked when Context is null.
 */
template <typename HandlerT, typename BuildFn>
void ReplyUnaryWithContext(HandlerT* handler, BuildFn&& build) {
    auto* context =
        handler->template GetUnsynchronizedContext<Context>();
    /**
     * @brief Decayed return type of @p build (the unary response protobuf).
     */
    using ResponseT = std::decay_t<decltype(build(context))>;
    if (!context) {
        ResponseT response{};
        if constexpr (detail::HasMutableStatus<ResponseT>::value) {
            *response.mutable_status() = ErrorStatus(
                ::automsgs::msgs::status_msgs::UNKNOWN, "no context");
        }
        ReplyUnary(handler, std::move(response));
        return;
    }
    ReplyUnary(handler, build(context));
}

/**
 * @brief Build a `common.Status` reply with Context, or UNKNOWN if missing.
 *
 * @details
 * Status-only counterpart of ReplyUnaryWithContext. Always emits a
 * `common.Status` body: UNKNOWN/"no context" when Context is null, otherwise
 * the Status returned by @p build(context).
 *
 * Used by BRIDGE_STATUS / BRIDGE_GOAL / BRIDGE_ACK and hand-written
 * Cancel/Pause/Resume OnRequest bodies.
 *
 * @tparam HandlerT async_grpc RpcHandler specialization.
 * @tparam BuildFn  Callable `(Context*) → common.Status`.
 * @param[in,out] handler Handler owning the reply path.
 * @param[in]     build   Status factory invoked with a live Context*; must
 *                        not retain the pointer past the call.
 *
 * @note @p build is not invoked when Context is null.
 */
template <typename HandlerT, typename BuildFn>
void ReplyStatusWithContext(HandlerT* handler, BuildFn&& build) {
    auto* context =
        handler->template GetUnsynchronizedContext<Context>();
    if (!context) {
        ReplyStatus(handler,
                    ErrorStatus(::automsgs::msgs::status_msgs::UNKNOWN,
                                  "no context"));
        return;
    }
    ReplyStatus(handler, build(context));
}

/**
 * @brief Build a stream sink that Send()s each frame and Finish()es on
 *        terminal.
 *
 * @details
 * Returns a callable `(const ResponseT&) → void` suitable as a stub
 * StreamCallback. Each invocation copies @p response into a unique_ptr,
 * Send()s it, and if @p is_final(response) finishes the RPC with OK.
 *
 * Prefer RelayStream for the common RequireContext + accepted-bool pattern;
 * use StreamUntil when the caller already holds a Context* and only needs
 * the sink.
 *
 * @tparam HandlerT  async_grpc RpcHandler specialization.
 * @tparam ResponseT Stream frame protobuf type.
 * @tparam IsFinalFn Predicate `bool(const ResponseT&)` — typically an
 *                   Is*Terminal helper below.
 * @param[in,out] handler  Handler owning Send / Finish for the lifetime of
 *                         the returned callback.
 * @param[in]     is_final Terminal-frame predicate (stored by value in the
 *                         lambda).
 * @return                 Callback suitable as a stub stream sink.
 *
 * @warning The returned lambda captures @p handler by raw pointer; the
 *          handler must outlive all sink invocations.
 */
template <typename HandlerT, typename ResponseT, typename IsFinalFn>
auto StreamUntil(HandlerT* handler, IsFinalFn&& is_final) {
    return [handler, is_final = std::forward<IsFinalFn>(is_final)](
               const ResponseT& response) {
        handler->Send(std::make_unique<ResponseT>(response));
        if (is_final(response)) {
            handler->Finish(::grpc::Status::OK);
        }
    };
}

/**
 * @brief Require Context, start a stub stream, and relay frames until
 *        terminal.
 *
 * @details
 * 1. RequireContext — abort with Finish(INTERNAL) if missing.
 * 2. Invoke @p call(context, sink) where @p sink Send()s each frame and
 *    Finish(OK) when @p is_final(frame).
 * 3. If @p call returns false (rejected before write) and @p reject_log is
 *    non-null, emit AWARN (stubs usually already emitted a reject frame).
 *
 * Used by BRIDGE_STREAM and hand-rolled Navigate / Record OnRequest paths.
 *
 * @tparam HandlerT  async_grpc RpcHandler specialization.
 * @tparam CallFn    Callable `(Context*, sink) → bool accepted` where sink
 *                   accepts each stream frame by const reference.
 * @tparam IsFinalFn Terminal-frame predicate on each response.
 * @param[in,out] handler     Handler owning the stream.
 * @param[in]     call        Stub start callable (e.g. lambda calling
 *                            `context->navigator().HandleNavigate`).
 * @param[in]     is_final    Finish predicate (e.g. &IsNavigateTerminal).
 * @param[in]     reject_log  Optional AWARN text when accepted==false;
 *                            nullptr suppresses the warning.
 *
 * @note Does not Finish when accepted==false unless the stub already closed
 *       the stream via the sink; callers that need an immediate Finish on
 *       reject should handle that inside @p call.
 */
template <typename HandlerT, typename CallFn, typename IsFinalFn>
void RelayStream(HandlerT* handler, CallFn&& call, IsFinalFn&& is_final,
                    const char* reject_log = nullptr) {
    auto* context = RequireContext(handler);
    if (!context) {
        return;
    }
    const bool accepted = std::forward<CallFn>(call)(
        context, [handler, is_final = std::forward<IsFinalFn>(is_final)](
                     const auto& response) {
            /**
             * @brief Decayed stream-frame type deduced from the stub sink
             *        argument.
             */
            using ResponseType = std::decay_t<decltype(response)>;
            handler->Send(std::make_unique<ResponseType>(response));
            if (is_final(response)) {
                handler->Finish(::grpc::Status::OK);
            }
        });
    if (!accepted && reject_log != nullptr) {
        AWARN << reject_log;
    }
}

/**
 * @brief Whether a NavigateResponse should end the Navigation stream.
 *
 * @details
 * Terminal when state is FAILED or CANCELLED. RUNNING / SUCCEEDED / other
 * non-failed states keep the stream open so progress frames continue.
 *
 * @param[in] response NavigationService stream frame.
 * @return             true ⇒ RelayStream / StreamUntil should Finish(OK).
 *
 * @see IsNavigateTerminal used by RpcNavigateHandler.
 */
inline bool IsNavigateTerminal(
    const ::automsgs::rpcs::navigation::NavigateResponse& response) {
    /**
     * @brief NavigationState enum alias for readability in the predicate.
     */
    using NavigationState = ::automsgs::rpcs::navigation::NavigationState;
    return response.state() == NavigationState::NAVIGATION_STATE_FAILED ||
           response.state() == NavigationState::NAVIGATION_STATE_CANCELLED;
}

/**
 * @brief Whether a FollowResponse should end the Follow stream.
 *
 * @details
 * Terminal on FAILED / CANCELLED, or IDLE with `active()==false` (session
 * ended cleanly without a failure code).
 *
 * @param[in] response FollowService stream frame.
 * @return             true ⇒ Finish the gRPC stream.
 */
inline bool IsFollowTerminal(
    const ::automsgs::rpcs::follow::FollowResponse& response) {
    /**
     * @brief FollowState enum alias for readability in the predicate.
     */
    using FollowState = ::automsgs::rpcs::follow::FollowState;
    return response.state() == FollowState::FOLLOW_STATE_FAILED ||
           response.state() == FollowState::FOLLOW_STATE_CANCELLED ||
           (response.state() == FollowState::FOLLOW_STATE_IDLE &&
            !response.active());
}

/**
 * @brief Whether a ChargeResponse should end the Charge stream.
 *
 * @details
 * Terminal on FAILED / CANCELLED / FULL (dock complete), or IDLE with
 * `active()==false`.
 *
 * @param[in] response ChargeService stream frame (Return / Leave).
 * @return             true ⇒ Finish the gRPC stream.
 */
inline bool IsChargeTerminal(
    const ::automsgs::rpcs::charge::ChargeResponse& response) {
    /**
     * @brief ChargeState enum alias for readability in the predicate.
     */
    using ChargeState = ::automsgs::rpcs::charge::ChargeState;
    return response.state() == ChargeState::CHARGE_STATE_FAILED ||
           response.state() == ChargeState::CHARGE_STATE_CANCELLED ||
           response.state() == ChargeState::CHARGE_STATE_FULL ||
           (response.state() == ChargeState::CHARGE_STATE_IDLE &&
            !response.active());
}

/**
 * @brief Whether an ExploreResponse should end the Exploration stream.
 *
 * @details
 * Terminal on FAILED / CANCELLED / COMPLETED (area covered / explore done).
 *
 * @param[in] response ExplorationService stream frame.
 * @return             true ⇒ Finish the gRPC stream.
 */
inline bool IsExploreTerminal(
    const ::automsgs::rpcs::exploration::ExploreResponse& response) {
    /**
     * @brief ExplorationState enum alias for readability in the predicate.
     */
    using ExplorationState = ::automsgs::rpcs::exploration::ExplorationState;
    return response.state() == ExplorationState::EXPLORATION_STATE_FAILED ||
           response.state() == ExplorationState::EXPLORATION_STATE_CANCELLED ||
           response.state() == ExplorationState::EXPLORATION_STATE_COMPLETED;
}

/**
 * @brief Whether a VoiceCommandResponse should end the Voice stream.
 *
 * @details
 * Terminal on FAILED / SUCCEEDED / CANCELLED.
 *
 * @param[in] response VoiceService stream frame.
 * @return             true ⇒ Finish the gRPC stream.
 */
inline bool IsVoiceTerminal(
    const ::automsgs::rpcs::voice::VoiceCommandResponse& response) {
    /**
     * @brief VoiceState enum alias for readability in the predicate.
     */
    using VoiceState = ::automsgs::rpcs::voice::VoiceState;
    return response.state() == VoiceState::VOICE_STATE_FAILED ||
           response.state() == VoiceState::VOICE_STATE_SUCCEEDED ||
           response.state() == VoiceState::VOICE_STATE_CANCELLED;
}

/**
 * @brief Whether a TeleopState value is a terminal / idle outcome.
 *
 * @details
 * Shared core for Teleop Velocity / DriveOnHeading / BackUp / Spin streams.
 * Treats SUCCEEDED, FAILED, CANCELLED, REJECTED, TIMEOUT, and IDLE as
 * terminal so the handler Finish()es after the final frame.
 *
 * @param[in] state TeleopState enum value from TeleopResponse::state().
 * @return          true ⇒ a teleop stream should Finish.
 */
inline bool IsTeleopTerminal(
    const ::automsgs::rpcs::teleop::TeleopState state) {
    /**
     * @brief TeleopState enum alias for readability in the predicate.
     */
    using TeleopState = ::automsgs::rpcs::teleop::TeleopState;
    return state == TeleopState::TELEOP_STATE_SUCCEEDED ||
           state == TeleopState::TELEOP_STATE_FAILED ||
           state == TeleopState::TELEOP_STATE_CANCELLED ||
           state == TeleopState::TELEOP_STATE_REJECTED ||
           state == TeleopState::TELEOP_STATE_TIMEOUT ||
           state == TeleopState::TELEOP_STATE_IDLE;
}

/**
 * @brief Whether a TeleopResponse should end the Teleop stream.
 *
 * @details
 * Overload that reads `response.state()` and forwards to
 * IsTeleopTerminal(TeleopState).
 *
 * @param[in] response TeleopService stream frame.
 * @return             true ⇒ Finish the gRPC stream.
 */
inline bool IsTeleopTerminal(
    const ::automsgs::rpcs::teleop::TeleopResponse& response) {
    return IsTeleopTerminal(response.state());
}

/**
 * @brief Alias of IsTeleopTerminal(TeleopResponse) for BRIDGE_STREAM slots.
 *
 * @details
 * Named distinctly so BRIDGE_STREAM can take `&IsTeleopResponseTerminal` as
 * the IsTerminalPredicate without overload-resolution ambiguity against the
 * TeleopState overload.
 *
 * @param[in] response TeleopService stream frame.
 * @return             true ⇒ Finish the gRPC stream.
 *
 * @see IsTeleopTerminal
 * @see BRIDGE_STREAM
 */
inline bool IsTeleopResponseTerminal(
    const ::automsgs::rpcs::teleop::TeleopResponse& response) {
    return IsTeleopTerminal(response);
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
