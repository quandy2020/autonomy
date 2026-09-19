/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_charge_handlers.hpp
 * @brief ChargeService RpcHandlers: Return/Leave streams, lifecycle, GetStatus.
 *
 * @details
 * Thin async_grpc handlers that forward to Context::charge() (ChargeStub).
 * Stub publishes on @c kChargingGoal / @c kChargingFeedback
 * (`/autonomy/task/charging/{goal,feedback}`). Handlers never touch Autolink
 * topics directly and never own session state.
 *
 * Generated types (each carries AUTONOMY_SMART_PTR_DEFINITIONS via macros):
 * - RpcChargeReturnHandler — BRIDGE_STREAM → HandleReturn / IsChargeTerminal
 * - RpcChargeLeaveHandler — BRIDGE_STREAM → HandleLeave / IsChargeTerminal
 * - RpcChargeCancelHandler / RpcChargePauseHandler / RpcChargeResumeHandler — BRIDGE_LIFECYCLE
 * - RpcChargeGetStatusHandler — BRIDGE_BUILD ActiveStatus (RETURNING vs IDLE)
 *
 * @par Invariants
 * - Streams finish on IsChargeTerminal.
 * - GetStatus uses ActiveStatus (RETURNING vs IDLE).
 * - Ownership: async_grpc instantiates handlers per RPC; stubs live in Context.
 * - Threading: OnRequest on the gRPC completion queue; stub may relay feedback
 *   asynchronously into the stream sink.
 *
 * @see ChargeStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/charge.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief ChargeService/Return — stream ChargeResponse until terminal. */
BRIDGE_STREAM(
    RpcChargeReturnHandler, ::automsgs::rpcs::charge::ReturnRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::charge::ChargeResponse>,
    "/automsgs.rpcs.charge.ChargeService/Return", &Context::charge,
    &clients::ChargeStub::HandleReturn, &IsChargeTerminal);

/** @brief ChargeService/Leave — undock stream until terminal. */
BRIDGE_STREAM(
    RpcChargeLeaveHandler, ::automsgs::rpcs::charge::LeaveRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::charge::ChargeResponse>,
    "/automsgs.rpcs.charge.ChargeService/Leave", &Context::charge,
    &clients::ChargeStub::HandleLeave, &IsChargeTerminal);

/**
 * @brief ChargeService Cancel / Pause / Resume on GoalRequest.goal_id.
 *
 * Expands to RpcChargeCancelHandler, RpcChargePauseHandler,
 * RpcChargeResumeHandler (each a GoalHandler typedef).
 */
BRIDGE_LIFECYCLE(RpcCharge, ::automsgs::rpcs::charge::GoalRequest,
                          "/automsgs.rpcs.charge.ChargeService", &Context::charge,
                          clients::ChargeStub);

/**
 * @brief ChargeService/GetStatus — ActiveStatus RETURNING when stub IsActive().
 */
BRIDGE_BUILD(
    RpcChargeGetStatusHandler, ::automsgs::rpcs::charge::GetStatusRequest,
    ::automsgs::rpcs::charge::ChargeResponse,
    "/automsgs.rpcs.charge.ChargeService/GetStatus",
    ActiveStatus<
        ::automsgs::rpcs::charge::ChargeResponse, &Context::charge,
        ::automsgs::rpcs::charge::CHARGE_STATE_RETURNING,
        ::automsgs::rpcs::charge::CHARGE_STATE_IDLE>);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
