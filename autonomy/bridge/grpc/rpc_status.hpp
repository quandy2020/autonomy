/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_status.hpp
 * @brief OkStatus / ErrorStatus — sole builders for automsgs.rpcs Status.
 *
 * @details
 * Domain stubs and handlers must build application-level Status via these
 * helpers so `status_msgs::StatusCode` mapping stays consistent across Charge,
 * Navigation, Map, Teleop, System, etc. Transport failures remain `grpc::Status`
 * at the async_grpc boundary and must not be constructed here.
 *
 * There are no wire topics in this header — Status is an envelope field inside
 * RPC response protos, not an Autolink channel.
 *
 * Invariants:
 * - Domain failures use status_msgs::StatusCode inside ErrorStatus.
 * - Reserve raw grpc::Status for transport / INTERNAL (e.g. missing Context).
 * - Do not invent parallel Status helpers in stubs or handlers.
 *
 * Ownership: none (inline free functions).
 * Threading: pure value builders; safe from any thread.
 *
 * @note No AUTONOMY_SMART_PTR_DEFINITIONS — this header declares no classes.
 * @see handler_templates.hpp
 */

#pragma once

#include <string>

#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/common.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Build an RPC Status from a status code and optional message.
 *
 * Sets Status::code from @p code and, when @p message is non-empty, copies it
 * into Status::message. Empty message leaves the protobuf field unset /
 * default.
 *
 * @param[in] code status_msgs::StatusCode (OK, CANCELLED, INVALID_ARGUMENT, …).
 * @param[in] message Optional human-readable detail for clients / logs.
 * @return Populated automsgs.rpcs.common.Status value.
 *
 * @note Prefer OkStatus() when code is OK so call sites read intent clearly.
 * @warning Do not overload this for grpc::Status; keep transport errors
 *          separate at the handler boundary.
 */
inline ::automsgs::rpcs::common::Status ErrorStatus(
    ::automsgs::msgs::status_msgs::StatusCode code,
    const std::string& message = "") {
    ::automsgs::rpcs::common::Status status;
    status.set_code(code);
    if (!message.empty()) {
        status.set_message(message);
    }
    return status;
}

/**
 * @brief Build an OK Status with an optional message.
 *
 * Convenience wrapper around ErrorStatus(status_msgs::OK, message).
 *
 * @param[in] message Optional success detail (usually empty; lifecycle handlers
 *            pass kCancelled / kPaused / kResumed).
 * @return Status with code OK.
 *
 * @note Success frames in streaming RPCs often embed OkStatus() inside the
 *       response envelope rather than returning grpc::Status::OK alone.
 */
inline ::automsgs::rpcs::common::Status OkStatus(
    const std::string& message = "") {
    return ErrorStatus(::automsgs::msgs::status_msgs::OK, message);
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
