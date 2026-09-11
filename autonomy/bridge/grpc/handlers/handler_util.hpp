/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared helpers for bridge gRPC RpcHandlers: cut repeated
 * GetUnsynchronizedContext / Send / Finish boilerplate.
 */

#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "autonomy/bridge/grpc/grpc_bridge_context.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/teleop.pb.h>
#include <grpc++/grpc++.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {
namespace detail {

template <typename T, typename = void>
struct HasMutableStatus : std::false_type {};

template <typename T>
struct HasMutableStatus<
    T, std::void_t<decltype(std::declval<T&>().mutable_status())>>
    : std::true_type {};

}  // namespace detail

template <typename HandlerT>
GrpcBridgeContextInterface* RequireContext(HandlerT* handler) {
    auto* context = handler->template GetUnsynchronizedContext<
        GrpcBridgeContextInterface>();
    if (!context) {
        handler->Finish(::grpc::Status(::grpc::StatusCode::INTERNAL,
                                       "no context"));
    }
    return context;
}

template <typename HandlerT, typename ResponseT>
void ReplyUnary(HandlerT* handler, std::unique_ptr<ResponseT> response) {
    handler->Send(std::move(response));
    handler->Finish(::grpc::Status::OK);
}

template <typename HandlerT, typename ResponseT>
void ReplyUnary(HandlerT* handler, ResponseT response) {
    ReplyUnary(handler, std::make_unique<ResponseT>(std::move(response)));
}

template <typename HandlerT>
void ReplyStatus(HandlerT* handler,
                 ::automsgs::rpcs::common::Status status) {
    ReplyUnary(handler, std::move(status));
}

/**
 * Unary with context. `build(context)` returns ResponseT by value.
 * When context is null and ResponseT has mutable_status(), fills UNKNOWN.
 */
template <typename HandlerT, typename BuildFn>
void ReplyUnaryWithContext(HandlerT* handler, BuildFn&& build) {
    auto* context =
        handler->template GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    using ResponseT = std::decay_t<decltype(build(context))>;
    if (!context) {
        ResponseT response{};
        if constexpr (detail::HasMutableStatus<ResponseT>::value) {
            *response.mutable_status() = MakeRpcStatus(
                ::automsgs::msgs::status_msgs::UNKNOWN, "no context");
        }
        ReplyUnary(handler, std::move(response));
        return;
    }
    ReplyUnary(handler, build(context));
}

template <typename HandlerT, typename BuildFn>
void ReplyStatusWithContext(HandlerT* handler, BuildFn&& build) {
    auto* context =
        handler->template GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (!context) {
        ReplyStatus(handler,
                    MakeRpcStatus(::automsgs::msgs::status_msgs::UNKNOWN,
                                  "no context"));
        return;
    }
    ReplyStatus(handler, build(context));
}

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

template <typename HandlerT, typename ResponseT>
auto StreamUntilAckFinal(HandlerT* handler) {
    return StreamUntil<HandlerT, ResponseT>(
        handler,
        [](const ResponseT& response) { return response.ack().final(); });
}

/**
 * AutonomyService command stream: require context, invoke stub HandleCommand
 * with a callback that finishes on ack.final().
 * `invoke(context, callback)` returns bool accepted.
 */
template <typename HandlerT, typename InvokeFn>
void RunCommandStream(HandlerT* handler, InvokeFn&& invoke,
                      const char* reject_log = nullptr) {
    auto* context = RequireContext(handler);
    if (!context) {
        return;
    }
    const bool accepted =
        std::forward<InvokeFn>(invoke)(context, [handler](const auto& response) {
            using ResponseT = std::decay_t<decltype(response)>;
            handler->Send(std::make_unique<ResponseT>(response));
            if (response.ack().final()) {
                handler->Finish(::grpc::Status::OK);
            }
        });
    if (!accepted && reject_log != nullptr) {
        AWARN << reject_log;
    }
}

/** Map bridge stream frames to RPC responses; finish on bridge ack.final(). */
template <typename HandlerT, typename MapFn>
auto StreamMappedUntilAckFinal(HandlerT* handler, MapFn&& map_response) {
    return [handler, map_response = std::forward<MapFn>(map_response)](
               const auto& bridge_response) {
        const bool final = bridge_response.ack().final();
        handler->Send(map_response(bridge_response));
        if (final) {
            handler->Finish(::grpc::Status::OK);
        }
    };
}

inline bool IsTeleopTerminal(
    const ::automsgs::rpcs::teleop::TeleopState state) {
    using TeleopState = ::automsgs::rpcs::teleop::TeleopState;
    return state == TeleopState::TELEOP_STATE_SUCCEEDED ||
           state == TeleopState::TELEOP_STATE_FAILED ||
           state == TeleopState::TELEOP_STATE_CANCELLED ||
           state == TeleopState::TELEOP_STATE_REJECTED ||
           state == TeleopState::TELEOP_STATE_TIMEOUT;
}

inline bool IsTeleopTerminal(
    const ::automsgs::rpcs::teleop::TeleopResponse& response) {
    return IsTeleopTerminal(response.state());
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
