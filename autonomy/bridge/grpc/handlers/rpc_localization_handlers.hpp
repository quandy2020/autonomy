/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/localization.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcLocGetPoseSignature, ::automsgs::rpcs::localization::GetPoseRequest,
    ::automsgs::rpcs::localization::GetPoseResponse,
    "/automsgs.rpcs.localization.LocalizationService/GetPose")

class RpcLocGetPoseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcLocGetPoseSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::localization::GetPoseRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcLocGetStatusSignature, ::automsgs::rpcs::localization::GetStatusRequest,
    ::automsgs::rpcs::localization::LocalizationStatus,
    "/automsgs.rpcs.localization.LocalizationService/GetStatus")

class RpcLocGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcLocGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::localization::GetStatusRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcLocSetInitialPoseSignature,
    ::automsgs::rpcs::localization::SetInitialPoseRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.localization.LocalizationService/SetInitialPose")

class RpcLocSetInitialPoseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcLocSetInitialPoseSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::localization::SetInitialPoseRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

