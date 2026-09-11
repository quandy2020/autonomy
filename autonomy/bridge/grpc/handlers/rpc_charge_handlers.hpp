/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/charge.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcChargeReturnSignature, ::automsgs::rpcs::charge::ReturnRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::charge::ChargeResponse>,
    "/automsgs.rpcs.charge.ChargeService/Return")

class RpcChargeReturnHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargeReturnSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::charge::ReturnRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcChargeLeaveSignature, ::automsgs::rpcs::charge::LeaveRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::charge::ChargeResponse>,
    "/automsgs.rpcs.charge.ChargeService/Leave")

class RpcChargeLeaveHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargeLeaveSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::charge::LeaveRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcChargeCancelSignature, ::automsgs::rpcs::charge::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.charge.ChargeService/Cancel")

class RpcChargeCancelHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargeCancelSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::charge::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcChargePauseSignature, ::automsgs::rpcs::charge::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.charge.ChargeService/Pause")

class RpcChargePauseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargePauseSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::charge::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcChargeResumeSignature, ::automsgs::rpcs::charge::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.charge.ChargeService/Resume")

class RpcChargeResumeHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargeResumeSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::charge::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcChargeGetStatusSignature, ::automsgs::rpcs::charge::GetStatusRequest,
    ::automsgs::rpcs::charge::ChargeResponse,
    "/automsgs.rpcs.charge.ChargeService/GetStatus")

class RpcChargeGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcChargeGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::charge::GetStatusRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

