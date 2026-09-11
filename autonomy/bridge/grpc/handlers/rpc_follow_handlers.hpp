/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/follow.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcFollowSignature, ::automsgs::rpcs::follow::FollowRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::follow::FollowResponse>,
    "/automsgs.rpcs.follow.FollowService/Follow")

class RpcFollowHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFollowSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::follow::FollowRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcFollowCancelSignature, ::automsgs::rpcs::follow::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.follow.FollowService/Cancel")

class RpcFollowCancelHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFollowCancelSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::follow::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcFollowPauseSignature, ::automsgs::rpcs::follow::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.follow.FollowService/Pause")

class RpcFollowPauseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFollowPauseSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::follow::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcFollowResumeSignature, ::automsgs::rpcs::follow::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.follow.FollowService/Resume")

class RpcFollowResumeHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFollowResumeSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::follow::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcFollowGetStatusSignature, ::automsgs::rpcs::follow::GetStatusRequest,
    ::automsgs::rpcs::follow::FollowResponse,
    "/automsgs.rpcs.follow.FollowService/GetStatus")

class RpcFollowGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFollowGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::follow::GetStatusRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

