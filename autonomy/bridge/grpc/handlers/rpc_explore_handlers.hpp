/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/exploration.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcExploreSignature, ::automsgs::rpcs::exploration::ExploreRequest,
    autonomy::common::async_grpc::Stream<
        ::automsgs::rpcs::exploration::ExploreResponse>,
    "/automsgs.rpcs.exploration.ExplorationService/Explore")

class RpcExploreHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::exploration::ExploreRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExploreCancelSignature, ::automsgs::rpcs::exploration::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.exploration.ExplorationService/Cancel")

class RpcExploreCancelHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreCancelSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::exploration::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExplorePauseSignature, ::automsgs::rpcs::exploration::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.exploration.ExplorationService/Pause")

class RpcExplorePauseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExplorePauseSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::exploration::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExploreResumeSignature, ::automsgs::rpcs::exploration::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.exploration.ExplorationService/Resume")

class RpcExploreResumeHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreResumeSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::exploration::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExploreGetStatusSignature, ::automsgs::rpcs::exploration::GetStatusRequest,
    ::automsgs::rpcs::exploration::ExploreResponse,
    "/automsgs.rpcs.exploration.ExplorationService/GetStatus")

class RpcExploreGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::exploration::GetStatusRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExploreSetAreaSignature, ::automsgs::rpcs::exploration::SetAreaRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.exploration.ExplorationService/SetArea")

class RpcExploreSetAreaHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreSetAreaSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::exploration::SetAreaRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcExploreSaveMapSignature, ::automsgs::rpcs::exploration::SaveMapRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.exploration.ExplorationService/SaveMap")

class RpcExploreSaveMapHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcExploreSaveMapSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::exploration::SaveMapRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

