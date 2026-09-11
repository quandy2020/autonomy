/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/command_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void SendFollowHandler::OnRequest(const proto::FollowCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->follow().HandleCommand(request, std::move(callback));
        },
        "SendFollowHandler: command rejected");
}

void SendDockHandler::OnRequest(const proto::DockCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->dock().HandleCommand(request, std::move(callback));
        },
        "SendDockHandler: command rejected");
}

void SendMapHandler::OnRequest(const proto::MapCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->map().HandleCommand(request, std::move(callback));
        },
        "SendMapHandler: command rejected");
}

void SendExplorationHandler::OnRequest(
    const proto::ExplorationCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->exploration().HandleCommand(request, std::move(callback));
        },
        "SendExplorationHandler: command rejected");
}

void SendVoiceHandler::OnRequest(const proto::VoiceCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->voice().HandleCommand(request, std::move(callback));
        },
        "SendVoiceHandler: command rejected");
}

void GetCapabilitiesHandler::OnRequest(const google::protobuf::Empty&) {
    proto::Capabilities response;
    response.set_supports_navigation(true);
    response.set_supports_follow(true);
    response.set_supports_teleop(true);
    response.set_supports_exploration(true);
    response.set_supports_docking(true);
    response.set_supports_map_management(true);
    response.set_bridge_version("autonomy.bridge");
    response.set_autonomy_version("autonomy");
    ReplyUnary(this, std::move(response));
}

void GetActiveTaskHandler::OnRequest(const google::protobuf::Empty&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->muxer().GetSnapshot();
    });
}

void EmergencyStopHandler::OnRequest(const proto::EmergencyStopRequest& request) {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    proto::CommandAck response;
    if (!context) {
        response.set_success(false);
        response.set_message("no context");
        response.set_final(true);
        ReplyUnary(this, std::move(response));
        return;
    }
    context->EmergencyStop(true);
    response.set_success(true);
    response.set_final(true);
    response.set_message(request.reason().empty() ? "estop engaged"
                                                  : request.reason());
    if (request.has_header()) {
        response.set_cmd_id(request.header().cmd_id());
    }
    ReplyUnary(this, std::move(response));
}

void CancelAllTasksHandler::OnRequest(
    const proto::CancelAllTasksRequest& request) {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    proto::CommandAck response;
    if (!context) {
        response.set_success(false);
        response.set_message("no context");
        response.set_final(true);
        ReplyUnary(this, std::move(response));
        return;
    }
    context->CancelAllTasks();
    response.set_success(true);
    response.set_final(true);
    response.set_message("cancelled");
    if (request.has_header()) {
        response.set_cmd_id(request.header().cmd_id());
    }
    ReplyUnary(this, std::move(response));
}

void GetRobotSnapshotHandler::OnRequest(const google::protobuf::Empty&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->state_hub().GetSnapshot();
    });
}

ReceiveBotStatesHandler::~ReceiveBotStatesHandler() {
    OnFinish();
}

void ReceiveBotStatesHandler::OnRequest(const google::protobuf::Empty&) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    Send(std::make_unique<::automsgs::msgs::vehicle_msgs::RobotState>(
        context->state_hub().GetSnapshot()));
    subscription_id_ = context->state_hub().SubscribeState(
        [this](const ::automsgs::msgs::vehicle_msgs::RobotState& state) {
            Send(std::make_unique<::automsgs::msgs::vehicle_msgs::RobotState>(
                state));
        });
}

void ReceiveBotStatesHandler::OnFinish() {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (context && subscription_id_ >= 0) {
        context->state_hub().Unsubscribe(subscription_id_);
        subscription_id_ = -1;
    }
}

ReceiveBotEventsHandler::~ReceiveBotEventsHandler() {
    OnFinish();
}

void ReceiveBotEventsHandler::OnRequest(const google::protobuf::Empty&) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    subscription_id_ = context->state_hub().SubscribeEvent(
        [this](const ::automsgs::msgs::vehicle_msgs::RobotEvent& event) {
            Send(std::make_unique<::automsgs::msgs::vehicle_msgs::RobotEvent>(
                event));
        });
}

void ReceiveBotEventsHandler::OnFinish() {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (context && subscription_id_ >= 0) {
        context->state_hub().Unsubscribe(subscription_id_);
        subscription_id_ = -1;
    }
}

void SendNavigationHandler::OnRequest(
    const proto::NavigationCommandRequest& request) {
    RunCommandStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->navigator().HandleCommand(request, std::move(callback));
        },
        "SendNavigationHandler: navigation command rejected.");
}

void SendTeleopHandler::OnRequest(const proto::TeleopCommandRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        stream_finished_ = true;
        return;
    }

    const bool accepted = context->teleop().HandleCommand(
        request, [this](const proto::TeleopCommandResponse& response) {
            Send(std::make_unique<proto::TeleopCommandResponse>(response));
            if (response.ack().final() && !stream_finished_) {
                stream_finished_ = true;
                Finish(::grpc::Status::OK);
            }
        });

    if (!accepted) {
        AWARN << "SendTeleopHandler: teleop command rejected immediately.";
    }
}

void SendTeleopHandler::OnReadsDone() {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (context) {
        context->teleop().ResetSession();
    }
    if (!stream_finished_) {
        stream_finished_ = true;
        Finish(::grpc::Status::OK);
    }
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

