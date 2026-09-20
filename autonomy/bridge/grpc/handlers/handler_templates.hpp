/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file handler_templates.hpp
 * @brief RpcHandler templates + BRIDGE_* macros (one line per RPC).
 *
 * @details
 * Handlers are intentionally thin: they parse nothing beyond what the
 * stub needs, never own domain state, and never touch Autolink topics
 * directly. Prefer the BRIDGE_* macros in domain `rpc_*_handlers.hpp`
 * files so Signature + handler typedef stay on one line.
 *
 * @par Invariants
 * - Handlers never own domain state; they only call Context → Stub.
 * - Stream handlers finish on IsTerminal; unary paths use OkStatus/ErrorStatus.
 * - Prefer BRIDGE_STREAM / LIFECYCLE / UNARY / DECL over hand-rolled classes.
 * - Every handler class / ActiveStatus / BRIDGE_DECL class carries
 * AUTONOMY_SMART_PTR_DEFINITIONS for SharedPtr / UniquePtr aliases.
 *
 * @see util.hpp
 * @see register_handlers.hpp
 */

#pragma once

#include <type_traits>
#include <utility>

#include "autonomy/bridge/grpc/handlers/util.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/common/macros.hpp"
#include <automsgs/rpcs/common.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief OkStatus message for Cancel lifecycle GoalHandlers. */
inline constexpr char kCancelled[] = "cancelled";
/** @brief OkStatus message for Pause lifecycle GoalHandlers. */
inline constexpr char kPaused[] = "paused";
/** @brief OkStatus message for Resume lifecycle GoalHandlers. */
inline constexpr char kResumed[] = "resumed";

/**
 * @brief Reply Status OK after running @p op(context).
 *
 * @tparam HandlerT   async_grpc RpcHandler type.
 * @tparam Op         Callable invoked with Context* when context is present.
 * @param[in,out] handler    Handler that owns the RPC reply path.
 * @param[in]     op         Stub side-effect (cancel / pause / ack).
 * @param[in]     ok_message Message embedded in OkStatus on success.
 */
template <typename HandlerT, typename Op>
void ReplyAck(HandlerT* handler, Op&& op, const char* ok_message) {
    ReplyStatusWithContext(handler, [&](auto* context) {
        std::forward<Op>(op)(context);
        return OkStatus(ok_message);
    });
}

/**
 * @brief Unary Status handler: stub.Method(goal_id) then OkStatus.
 *
 * Selects the domain stub via @p Getter on Context, then calls @p Method with
 * `request.goal_id()`. Typical use: Cancel / Pause / Resume RPCs.
 *
 * @tparam Signature  async_grpc handler signature (request/response/path).
 * @tparam Getter     Pointer-to-member: Context → domain stub.
 * @tparam Method     Pointer-to-member: stub → void(goal_id) (or compatible).
 * @tparam kOkMessage Compile-time OK status message string.
 */
template <typename Signature, auto Getter, auto Method,
          const char* kOkMessage>
class GoalHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        GoalHandler<Signature, Getter, Method, kOkMessage>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Invoke stub Method(goal_id) then reply OkStatus(@p kOkMessage).
     *
     * @param[in] request GoalRequest-like message exposing goal_id().
     */
    void OnRequest(const RequestType& request) override {
        ReplyAck(
            this,
            [&](auto* context) {
                ((context->*Getter)().*Method)(request.goal_id());
            },
            kOkMessage);
    }
};

/**
 * @brief Unary→Stream handler: stub.Handle*(request, callback).
 *
 * Relays frames until @p IsTerminal(frame) is true, then finishes OK.
 * Logs ingress (goal_id) before accepting the stream.
 *
 * @tparam Signature  async_grpc handler signature.
 * @tparam Getter     Pointer-to-member: Context → domain stub.
 * @tparam Method     Pointer-to-member: stub stream start (request, callback).
 * @tparam IsTerminal Predicate on each response frame.
 */
template <typename Signature, auto Getter, auto Method, auto IsTerminal>
class StreamHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        StreamHandler<Signature, Getter, Method, IsTerminal>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Log ingress then RelayStream via stub Method until IsTerminal.
     *
     * @param[in] request Domain start request forwarded to the stub.
     */
    void OnRequest(const RequestType& request) override {
        LogIngress(Signature::MethodName(), request);
        RelayStream(
            this,
            [&](auto* context, auto&& callback) {
                return ((context->*Getter)().*Method)(request,
                                                          std::move(callback));
            },
            IsTerminal);
    }
};

/**
 * @brief Unary Status OK after void stub.Method(request).
 *
 * Unlike GoalHandler, passes the full request protobuf to the stub method.
 *
 * @tparam Signature  async_grpc handler signature.
 * @tparam Getter     Pointer-to-member: Context → domain stub.
 * @tparam Method     Pointer-to-member: stub → void(request).
 * @tparam kOkMessage Compile-time OK status message string.
 */
template <typename Signature, auto Getter, auto Method,
          const char* kOkMessage>
class AckHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        AckHandler<Signature, Getter, Method, kOkMessage>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Invoke stub Method(request) then reply OkStatus(@p kOkMessage).
     *
     * @param[in] request Full request protobuf passed to the stub.
     */
    void OnRequest(const RequestType& request) override {
        ReplyAck(
            this,
            [&](auto* context) {
                ((context->*Getter)().*Method)(request);
            },
            kOkMessage);
    }
};

/**
 * @brief Unary handler: stub.Method(request) → response protobuf.
 *
 * @tparam Signature async_grpc handler signature.
 * @tparam Getter    Pointer-to-member: Context → domain stub.
 * @tparam Method    Pointer-to-member: stub → Response(request).
 */
template <typename Signature, auto Getter, auto Method>
class UnaryHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        UnaryHandler<Signature, Getter, Method>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Reply with stub Method(request) as the unary response body.
     *
     * @param[in] request Request forwarded to the stub method.
     */
    void OnRequest(const RequestType& request) override {
        ReplyUnaryWithContext(this, [&](auto* context) {
            return ((context->*Getter)().*Method)(request);
        });
    }
};

/**
 * @brief Unary handler: stub.Method() → response (request fields unused).
 *
 * Typical for GetStatus / List* RPCs that ignore the request body.
 *
 * @tparam Signature async_grpc handler signature.
 * @tparam Getter    Pointer-to-member: Context → domain stub.
 * @tparam Method    Pointer-to-member: stub → Response().
 */
template <typename Signature, auto Getter, auto Method>
class GetHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(GetHandler<Signature, Getter, Method>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base (unused body). */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Reply with stub Method() ignoring the request body.
     *
     * @param[in] request Ignored; present for RpcHandler signature.
     */
    void OnRequest(const RequestType&) override {
        ReplyUnaryWithContext(this, [](auto* context) {
            return ((context->*Getter)().*Method)();
        });
    }
};

/**
 * @brief Unary Status handler: stub.Method(request) returns Status.
 *
 * Forwards the stub Status as the RPC response body (common.Status).
 *
 * @tparam Signature async_grpc handler signature.
 * @tparam Getter    Pointer-to-member: Context → domain stub.
 * @tparam Method    Pointer-to-member: stub → Status(request).
 */
template <typename Signature, auto Getter, auto Method>
class StatusHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        StatusHandler<Signature, Getter, Method>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Reply with stub Method(request) as common.Status.
     *
     * @param[in] request Request forwarded to the stub method.
     */
    void OnRequest(const RequestType& request) override {
        ReplyStatusWithContext(this, [&](auto* context) {
            return ((context->*Getter)().*Method)(request);
        });
    }
};

/**
 * @brief Unary response with mutable_status() filled from stub Status.
 *
 * Builds a default ResponseType, assigns stub.Method(request) into
 * `response.mutable_status()`, and sends the wrapped response.
 *
 * @tparam Signature async_grpc handler signature.
 * @tparam Getter    Pointer-to-member: Context → domain stub.
 * @tparam Method    Pointer-to-member: stub → Status(request).
 */
template <typename Signature, auto Getter, auto Method>
class WrapHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        WrapHandler<Signature, Getter, Method>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base. */
    using RequestType = typename Base::RequestType;
    /** @brief Outgoing response protobuf type from @ref Base. */
    using ResponseType = typename Base::ResponseType;

    /**
     * @brief Wrap stub Status into response.mutable_status() and reply.
     *
     * @param[in] request Request forwarded to the stub method.
     */
    void OnRequest(const RequestType& request) override {
        ReplyUnaryWithContext(this, [&](auto* context) {
            ResponseType response{};
            *response.mutable_status() =
                ((context->*Getter)().*Method)(request);
            return response;
        });
    }
};

/**
 * @brief Unary handler that delegates response construction to Builder::Build.
 *
 * @tparam Signature async_grpc handler signature.
 * @tparam Builder   Type with `static Response Build(Context*)`.
 */
template <typename Signature, typename Builder>
class BuildHandler
    : public autonomy::common::async_grpc::RpcHandler<Signature>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(BuildHandler<Signature, Builder>)

    /** @brief Base RpcHandler specialization for @p Signature. */
    using Base = autonomy::common::async_grpc::RpcHandler<Signature>;
    /** @brief Incoming request protobuf type from @ref Base (unused body). */
    using RequestType = typename Base::RequestType;

    /**
     * @brief Reply with Builder::Build(context), ignoring the request body.
     *
     * @param[in] request Ignored; present for RpcHandler signature.
     */
    void OnRequest(const RequestType&) override {
        ReplyUnaryWithContext(this, &Builder::Build);
    }
};

/**
 * @brief GetStatus-style builder: active flag + state enum from IsActive().
 *
 * Sets status=OK, `active` from stub.IsActive(), and `state` to
 * @p kActiveState or @p kIdleState accordingly.
 *
 * @tparam Response     Response protobuf type with set_active / set_state.
 * @tparam Getter       Pointer-to-member: Context → domain stub.
 * @tparam kActiveState State enum value when active.
 * @tparam kIdleState   State enum value when idle.
 */
template <typename Response, auto Getter, auto kActiveState,
          auto kIdleState>
struct ActiveStatus {
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        ActiveStatus<Response, Getter, kActiveState, kIdleState>)

    /**
     * @brief Build active/idle status snapshot from stub IsActive().
     *
     * @param[in] context Bridge Context providing the domain stub via Getter.
     * @return            Response with OK status, active flag, and state enum.
     */
    static Response Build(Context* context) {
        Response response;
        *response.mutable_status() = OkStatus();
        const bool active = (context->*Getter)().IsActive();
        response.set_active(active);
        response.set_state(active ? kActiveState : kIdleState);
        return response;
    }
};

// ---------------------------------------------------------------------------
// Macros: Signature + using / class in one shot
// ---------------------------------------------------------------------------

/**
 * @def BRIDGE_STREAM
 * @brief Define Signature + `using HandlerName = StreamHandler<…>`.
 *
 * @param HandlerName          Handler typedef name (also defines
 *                             HandlerName##Signature).
 * @param IncomingType         Request type (often a unary request protobuf).
 * @param OutgoingType         Response stream type
 *                             (`async_grpc::Stream<Response>`).
 * @param MethodPath           Fully-qualified gRPC method path string.
 * @param ContextGetter        `&Context::domain` pointer-to-member.
 * @param StubMethod           `&Stub::Handle*` stream-start pointer-to-member.
 * @param IsTerminalPredicate  Predicate `bool(const Response&)` for Finish.
 */
#define BRIDGE_STREAM(HandlerName, IncomingType, OutgoingType, MethodPath,        \
                      ContextGetter, StubMethod, IsTerminalPredicate)             \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    using HandlerName = ::autonomy::bridge::grpc::handlers::StreamHandler<        \
        HandlerName##Signature, ContextGetter, StubMethod, IsTerminalPredicate>

/**
 * @def BRIDGE_GOAL
 * @brief Define Signature + `using HandlerName = GoalHandler<…>` (Status OK).
 *
 * @param HandlerName       Handler typedef name.
 * @param GoalRequestType   GoalRequest-like protobuf with goal_id().
 * @param MethodPath        Fully-qualified gRPC method path string.
 * @param ContextGetter     `&Context::domain` pointer-to-member.
 * @param StubMethod        `&Stub::CancelGoal` / Pause / Resume
 *                          pointer-to-member.
 * @param OkStatusMessage   Compile-time `const char*` OkStatus message.
 */
#define BRIDGE_GOAL(HandlerName, GoalRequestType, MethodPath, ContextGetter,      \
                    StubMethod, OkStatusMessage)                                  \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, GoalRequestType,             \
                             ::automsgs::rpcs::common::Status, MethodPath)        \
    using HandlerName = ::autonomy::bridge::grpc::handlers::GoalHandler<          \
        HandlerName##Signature, ContextGetter, StubMethod, OkStatusMessage>

/**
 * @def BRIDGE_LIFECYCLE
 * @brief Expand Cancel + Pause + Resume GoalHandlers for one service.
 *
 * Produces `HandlerNamePrefixCancelHandler`,
 * `HandlerNamePrefixPauseHandler`, `HandlerNamePrefixResumeHandler`.
 *
 * @param HandlerNamePrefix  Name prefix (e.g. RpcNavigation →
 *                           RpcNavigationCancelHandler).
 * @param GoalRequestType    GoalRequest protobuf type.
 * @param ServicePath        Service path prefix without trailing method
 *                           (e.g. "/….NavigationService").
 * @param ContextGetter      `&Context::domain` pointer-to-member.
 * @param StubType           Stub type providing CancelGoal / PauseGoal /
 *                           ResumeGoal.
 */
#define BRIDGE_LIFECYCLE(HandlerNamePrefix, GoalRequestType, ServicePath,         \
                         ContextGetter, StubType)                                 \
    BRIDGE_GOAL(HandlerNamePrefix##CancelHandler, GoalRequestType,                \
                ServicePath "/Cancel", ContextGetter, &StubType::CancelGoal,      \
                ::autonomy::bridge::grpc::handlers::kCancelled);                  \
    BRIDGE_GOAL(HandlerNamePrefix##PauseHandler, GoalRequestType,                 \
                ServicePath "/Pause", ContextGetter, &StubType::PauseGoal,        \
                ::autonomy::bridge::grpc::handlers::kPaused);                     \
    BRIDGE_GOAL(HandlerNamePrefix##ResumeHandler, GoalRequestType,                \
                ServicePath "/Resume", ContextGetter, &StubType::ResumeGoal,      \
                ::autonomy::bridge::grpc::handlers::kResumed)

/**
 * @def BRIDGE_ACK
 * @brief Define Signature + `using HandlerName = AckHandler<…>` (void → OkStatus).
 *
 * @param HandlerName       Handler typedef name.
 * @param IncomingType      Request protobuf type.
 * @param MethodPath        Fully-qualified gRPC method path string.
 * @param ContextGetter     `&Context::domain` pointer-to-member.
 * @param StubMethod        `&Stub::Method` void(request) pointer-to-member.
 * @param OkStatusMessage   Compile-time `const char*` OkStatus message.
 */
#define BRIDGE_ACK(HandlerName, IncomingType, MethodPath, ContextGetter,          \
                   StubMethod, OkStatusMessage)                                   \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType,                \
                             ::automsgs::rpcs::common::Status, MethodPath)        \
    using HandlerName = ::autonomy::bridge::grpc::handlers::AckHandler<           \
        HandlerName##Signature, ContextGetter, StubMethod, OkStatusMessage>

/**
 * @def BRIDGE_UNARY
 * @brief Define Signature + `using HandlerName = UnaryHandler<…>`.
 *
 * @param HandlerName     Handler typedef name.
 * @param IncomingType    Request protobuf type.
 * @param OutgoingType    Response protobuf type.
 * @param MethodPath      Fully-qualified gRPC method path string.
 * @param ContextGetter   `&Context::domain` pointer-to-member.
 * @param StubMethod      `&Stub::Method` returning OutgoingType from
 *                        IncomingType.
 */
#define BRIDGE_UNARY(HandlerName, IncomingType, OutgoingType, MethodPath,         \
                     ContextGetter, StubMethod)                                   \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    using HandlerName = ::autonomy::bridge::grpc::handlers::UnaryHandler<         \
        HandlerName##Signature, ContextGetter, StubMethod>

/**
 * @def BRIDGE_GET
 * @brief Define Signature + `using HandlerName = GetHandler<…>` (ignore request).
 *
 * @param HandlerName     Handler typedef name.
 * @param IncomingType    Request protobuf type (body unused).
 * @param OutgoingType    Response protobuf type.
 * @param MethodPath      Fully-qualified gRPC method path string.
 * @param ContextGetter   `&Context::domain` pointer-to-member.
 * @param StubMethod      `&Stub::Method` returning OutgoingType with no args.
 */
#define BRIDGE_GET(HandlerName, IncomingType, OutgoingType, MethodPath,           \
                   ContextGetter, StubMethod)                                     \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    using HandlerName = ::autonomy::bridge::grpc::handlers::GetHandler<           \
        HandlerName##Signature, ContextGetter, StubMethod>

/**
 * @def BRIDGE_STATUS
 * @brief Define Signature + `using HandlerName = StatusHandler<…>`.
 *
 * @param HandlerName     Handler typedef name.
 * @param IncomingType    Request protobuf type.
 * @param MethodPath      Fully-qualified gRPC method path string.
 * @param ContextGetter   `&Context::domain` pointer-to-member.
 * @param StubMethod      `&Stub::Method` returning common.Status.
 */
#define BRIDGE_STATUS(HandlerName, IncomingType, MethodPath, ContextGetter,       \
                      StubMethod)                                                 \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType,                \
                             ::automsgs::rpcs::common::Status, MethodPath)        \
    using HandlerName = ::autonomy::bridge::grpc::handlers::StatusHandler<        \
        HandlerName##Signature, ContextGetter, StubMethod>

/**
 * @def BRIDGE_WRAP
 * @brief Define Signature + `using HandlerName = WrapHandler<…>` (Status → field).
 *
 * @param HandlerName     Handler typedef name.
 * @param IncomingType    Request protobuf type.
 * @param OutgoingType    Response with mutable_status().
 * @param MethodPath      Fully-qualified gRPC method path string.
 * @param ContextGetter   `&Context::domain` pointer-to-member.
 * @param StubMethod      `&Stub::Method` returning common.Status.
 */
#define BRIDGE_WRAP(HandlerName, IncomingType, OutgoingType, MethodPath,          \
                    ContextGetter, StubMethod)                                    \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    using HandlerName = ::autonomy::bridge::grpc::handlers::WrapHandler<          \
        HandlerName##Signature, ContextGetter, StubMethod>

/**
 * @def BRIDGE_BUILD
 * @brief Define Signature + `using HandlerName = BuildHandler<…>`.
 *
 * @param HandlerName       Handler typedef name.
 * @param IncomingType      Request protobuf type (body unused).
 * @param OutgoingType      Response protobuf type.
 * @param MethodPath        Fully-qualified gRPC method path string.
 * @param ...               ResponseBuilder type (`static OutgoingType Build(Context*)`).
 *                          Variadic so `ActiveStatus<A, B, C, D>` commas are preserved.
 */
#define BRIDGE_BUILD(HandlerName, IncomingType, OutgoingType, MethodPath, ...)    \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    using HandlerName = ::autonomy::bridge::grpc::handlers::BuildHandler<         \
        HandlerName##Signature, __VA_ARGS__>

/**
 * @def BRIDGE_DECL
 * @brief Declare a hand-written RpcHandler class (OnRequest in .cpp / inline).
 *
 * Expands to `class HandlerName` with
 * AUTONOMY_SMART_PTR_DEFINITIONS(HandlerName) in the public section, plus a
 * virtual OnRequest declaration.
 *
 * @param HandlerName    Handler class name.
 * @param IncomingType   Request type for OnRequest.
 * @param OutgoingType   Response / stream type for the Signature.
 * @param MethodPath     Fully-qualified gRPC method path string.
 */
#define BRIDGE_DECL(HandlerName, IncomingType, OutgoingType, MethodPath)          \
    DEFINE_HANDLER_SIGNATURE(HandlerName##Signature, IncomingType, OutgoingType,  \
                             MethodPath)                                          \
    class HandlerName                                                             \
        : public autonomy::common::async_grpc::RpcHandler<                        \
              HandlerName##Signature> {                                           \
    public:                                                                       \
        AUTONOMY_SMART_PTR_DEFINITIONS(HandlerName)                               \
        void OnRequest(const IncomingType& request) override;                     \
    }

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
