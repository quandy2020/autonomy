/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/bridge/proto/external_command_service.grpc.pb.h"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "google/protobuf/empty.pb.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    SendNavigationSignature, proto::NavigationCommandRequest,
    autonomy::common::async_grpc::Stream<proto::NavigationCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendNavigationCommand")

class SendNavigationHandler
    : public autonomy::common::async_grpc::RpcHandler<SendNavigationSignature>
{
public:
    void OnRequest(const proto::NavigationCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    SendTeleopSignature,
    autonomy::common::async_grpc::Stream<proto::TeleopCommandRequest>,
    autonomy::common::async_grpc::Stream<proto::TeleopCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendTeleopCommand")

class SendTeleopHandler
    : public autonomy::common::async_grpc::RpcHandler<SendTeleopSignature>
{
public:
    void OnRequest(const proto::TeleopCommandRequest& request) override;
    void OnReadsDone() override;

private:
    bool stream_finished_{false};
};

DEFINE_HANDLER_SIGNATURE(
    SendFollowSignature, proto::FollowCommandRequest,
    autonomy::common::async_grpc::Stream<proto::FollowCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendFollowCommand")

class SendFollowHandler
    : public autonomy::common::async_grpc::RpcHandler<SendFollowSignature>
{
public:
    void OnRequest(const proto::FollowCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    SendDockSignature, proto::DockCommandRequest,
    autonomy::common::async_grpc::Stream<proto::DockCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendDockCommand")

class SendDockHandler
    : public autonomy::common::async_grpc::RpcHandler<SendDockSignature>
{
public:
    void OnRequest(const proto::DockCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    SendMapSignature, proto::MapCommandRequest,
    autonomy::common::async_grpc::Stream<proto::MapCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendMapCommand")

class SendMapHandler
    : public autonomy::common::async_grpc::RpcHandler<SendMapSignature>
{
public:
    void OnRequest(const proto::MapCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    SendExplorationSignature, proto::ExplorationCommandRequest,
    autonomy::common::async_grpc::Stream<proto::ExplorationCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendExplorationCommand")

class SendExplorationHandler
    : public autonomy::common::async_grpc::RpcHandler<SendExplorationSignature>
{
public:
    void OnRequest(const proto::ExplorationCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    SendVoiceSignature, proto::VoiceCommandRequest,
    autonomy::common::async_grpc::Stream<proto::VoiceCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendVoiceCommand")

class SendVoiceHandler
    : public autonomy::common::async_grpc::RpcHandler<SendVoiceSignature>
{
public:
    void OnRequest(const proto::VoiceCommandRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    GetCapabilitiesSignature, google::protobuf::Empty, proto::Capabilities,
    "/autonomy.bridge.proto.AutonomyService/GetCapabilities")

class GetCapabilitiesHandler
    : public autonomy::common::async_grpc::RpcHandler<GetCapabilitiesSignature>
{
public:
    void OnRequest(const google::protobuf::Empty& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    GetActiveTaskSignature, google::protobuf::Empty, proto::ActiveTaskInfo,
    "/autonomy.bridge.proto.AutonomyService/GetActiveTask")

class GetActiveTaskHandler
    : public autonomy::common::async_grpc::RpcHandler<GetActiveTaskSignature>
{
public:
    void OnRequest(const google::protobuf::Empty& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    GetRobotSnapshotSignature, google::protobuf::Empty,
    ::automsgs::msgs::vehicle_msgs::RobotState,
    "/autonomy.bridge.proto.AutonomyService/GetRobotSnapshot")

class GetRobotSnapshotHandler
    : public autonomy::common::async_grpc::RpcHandler<GetRobotSnapshotSignature>
{
public:
    void OnRequest(const google::protobuf::Empty& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    ReceiveBotStatesSignature, google::protobuf::Empty,
    autonomy::common::async_grpc::Stream<::automsgs::msgs::vehicle_msgs::RobotState>,
    "/autonomy.bridge.proto.AutonomyService/ReceiveBotStates")

class ReceiveBotStatesHandler
    : public autonomy::common::async_grpc::RpcHandler<ReceiveBotStatesSignature>
{
public:
    ~ReceiveBotStatesHandler() override;
    void OnRequest(const google::protobuf::Empty& request) override;
    void OnFinish() override;

private:
    int64_t subscription_id_{-1};
};

DEFINE_HANDLER_SIGNATURE(
    ReceiveBotEventsSignature, google::protobuf::Empty,
    autonomy::common::async_grpc::Stream<::automsgs::msgs::vehicle_msgs::RobotEvent>,
    "/autonomy.bridge.proto.AutonomyService/ReceiveBotEvents")

class ReceiveBotEventsHandler
    : public autonomy::common::async_grpc::RpcHandler<ReceiveBotEventsSignature>
{
public:
    ~ReceiveBotEventsHandler() override;
    void OnRequest(const google::protobuf::Empty& request) override;
    void OnFinish() override;

private:
    int64_t subscription_id_{-1};
};

DEFINE_HANDLER_SIGNATURE(
    EmergencyStopSignature, proto::EmergencyStopRequest, proto::CommandAck,
    "/autonomy.bridge.proto.AutonomyService/EmergencyStop")

class EmergencyStopHandler
    : public autonomy::common::async_grpc::RpcHandler<EmergencyStopSignature>
{
public:
    void OnRequest(const proto::EmergencyStopRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    CancelAllTasksSignature, proto::CancelAllTasksRequest, proto::CommandAck,
    "/autonomy.bridge.proto.AutonomyService/CancelAllTasks")

class CancelAllTasksHandler
    : public autonomy::common::async_grpc::RpcHandler<CancelAllTasksSignature>
{
public:
    void OnRequest(const proto::CancelAllTasksRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
