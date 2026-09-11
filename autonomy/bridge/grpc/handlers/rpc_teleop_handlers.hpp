/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>
#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/teleop.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcTeleopVelocitySignature,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::teleop::VelocityRequest>,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::teleop::TeleopResponse>,
    "/automsgs.rpcs.teleop.TeleopService/Velocity")

class RpcTeleopVelocityHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcTeleopVelocitySignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::teleop::VelocityRequest& request) override;
    void OnReadsDone() override;

private:
    bool stream_finished_{false};
    std::string goal_id_;
};

DEFINE_HANDLER_SIGNATURE(
    RpcDriveOnHeadingSignature, ::automsgs::rpcs::teleop::DriveOnHeadingRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::teleop::TeleopResponse>,
    "/automsgs.rpcs.teleop.TeleopService/DriveOnHeading")

class RpcDriveOnHeadingHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcDriveOnHeadingSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::DriveOnHeadingRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcBackUpSignature, ::automsgs::rpcs::teleop::BackUpRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::teleop::TeleopResponse>,
    "/automsgs.rpcs.teleop.TeleopService/BackUp")

class RpcBackUpHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcBackUpSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::BackUpRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSpinSignature, ::automsgs::rpcs::teleop::SpinRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::teleop::TeleopResponse>,
    "/automsgs.rpcs.teleop.TeleopService/Spin")

class RpcSpinHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSpinSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::SpinRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcTeleopCancelSignature, ::automsgs::rpcs::teleop::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.teleop.TeleopService/Cancel")

class RpcTeleopCancelHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcTeleopCancelSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcTeleopPauseSignature, ::automsgs::rpcs::teleop::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.teleop.TeleopService/Pause")

class RpcTeleopPauseHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcTeleopPauseSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcTeleopResumeSignature, ::automsgs::rpcs::teleop::GoalRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.teleop.TeleopService/Resume")

class RpcTeleopResumeHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcTeleopResumeSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::GoalRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcTeleopGetStatusSignature, ::automsgs::rpcs::teleop::GetStatusRequest,
    ::automsgs::rpcs::teleop::TeleopResponse,
    "/automsgs.rpcs.teleop.TeleopService/GetStatus")

class RpcTeleopGetStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcTeleopGetStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::teleop::GetStatusRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

