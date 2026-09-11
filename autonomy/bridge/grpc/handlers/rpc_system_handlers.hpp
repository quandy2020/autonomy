/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcHeartbeatSignature, ::automsgs::rpcs::system::HeartbeatRequest,
    ::automsgs::rpcs::system::HeartbeatResponse,
    "/automsgs.rpcs.system.SystemService/Heartbeat")

class RpcHeartbeatHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcHeartbeatSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::HeartbeatRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemGetInfoSignature, ::automsgs::rpcs::system::GetInfoRequest,
    ::automsgs::rpcs::system::GetInfoResponse,
    "/automsgs.rpcs.system.SystemService/GetInfo")

class RpcSystemGetInfoHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSystemGetInfoSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::system::GetInfoRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemGetStatusSignature, ::automsgs::rpcs::system::GetStatusRequest,
    ::automsgs::rpcs::system::GetStatusResponse,
    "/automsgs.rpcs.system.SystemService/GetStatus")

class RpcSystemGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSystemGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::system::GetStatusRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemGetHealthSignature, ::automsgs::rpcs::system::GetHealthRequest,
    ::automsgs::rpcs::system::GetHealthResponse,
    "/automsgs.rpcs.system.SystemService/GetHealth")

class RpcSystemGetHealthHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSystemGetHealthSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::system::GetHealthRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemCapabilitiesSignature,
    ::automsgs::rpcs::system::GetCapabilitiesRequest,
    ::automsgs::rpcs::system::Capabilities,
    "/automsgs.rpcs.system.SystemService/GetCapabilities")

class RpcSystemCapabilitiesHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSystemCapabilitiesSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::GetCapabilitiesRequest& request)
        override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemEstopSignature, ::automsgs::rpcs::system::EmergencyStopRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.system.SystemService/EmergencyStop")

class RpcSystemEstopHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSystemEstopSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::EmergencyStopRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemClearEstopSignature,
    ::automsgs::rpcs::system::ClearEmergencyStopRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.system.SystemService/ClearEmergencyStop")

class RpcSystemClearEstopHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSystemClearEstopSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::ClearEmergencyStopRequest& request)
        override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemCancelAllSignature, ::automsgs::rpcs::system::CancelAllGoalsRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.system.SystemService/CancelAllGoals")

class RpcSystemCancelAllHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSystemCancelAllSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::CancelAllGoalsRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSystemActiveGoalSignature, ::automsgs::rpcs::system::GetActiveGoalRequest,
    ::automsgs::rpcs::system::ActiveGoal,
    "/automsgs.rpcs.system.SystemService/GetActiveGoal")

class RpcSystemActiveGoalHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSystemActiveGoalSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::system::GetActiveGoalRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

