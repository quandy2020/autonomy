/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/navigation.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcNavigateSignature, ::automsgs::rpcs::navigation::NavigateRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::navigation::NavigateResponse>,
    "/automsgs.rpcs.navigation.NavigationService/Navigate")

class RpcNavigateHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcNavigateSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::navigation::NavigateRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcNavCancelSignature, ::automsgs::rpcs::navigation::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.navigation.NavigationService/Cancel")

class RpcNavCancelHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcNavCancelSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::navigation::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcNavPauseSignature, ::automsgs::rpcs::navigation::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.navigation.NavigationService/Pause")

class RpcNavPauseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcNavPauseSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::navigation::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcNavResumeSignature, ::automsgs::rpcs::navigation::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.navigation.NavigationService/Resume")

class RpcNavResumeHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcNavResumeSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::navigation::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcNavGetStatusSignature, ::automsgs::rpcs::navigation::GetStatusRequest,
    ::automsgs::rpcs::navigation::NavigateResponse,
    "/automsgs.rpcs.navigation.NavigationService/GetStatus")

class RpcNavGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcNavGetStatusSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::navigation::GetStatusRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

