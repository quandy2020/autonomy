/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/teleop_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/bridge/grpc/teleop_goal_convert.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;

bool CheckTeleopTerminalStatus(task_proto::TeleopStatus status) {
    return status == task_proto::TELEOP_STATUS_TIMEOUT ||
           status == task_proto::TELEOP_STATUS_REJECTED ||
           status == task_proto::TELEOP_STATUS_IDLE;
}

proto::TaskStatus ResolveTeleopTaskStatus(proto::TeleopStatus status,
                                          bool success) {
    switch (status) {
        case proto::TELEOP_STATUS_ACTIVE:
            return proto::TASK_STATUS_RUNNING;
        case proto::TELEOP_STATUS_IDLE:
            return proto::TASK_STATUS_IDLE;
        case proto::TELEOP_STATUS_TIMEOUT:
        case proto::TELEOP_STATUS_REJECTED:
            return success ? proto::TASK_STATUS_IDLE : proto::TASK_STATUS_FAILED;
        default:
            return proto::TASK_STATUS_UNKNOWN;
    }
}

TeleopTraits::Response MakeTeleopStatusResponse(
    const TeleopTraits::Request& request, proto::TeleopStatus status,
    bool success, bool final, const std::string& message = "") {
    TeleopTraits::Response response;
    response.set_status(status);
    FillCommandAck(response, TeleopTraits::kTaskType, request, success, final,
                   ResolveTeleopTaskStatus(status, success), message);
    return response;
}

}  // namespace

TeleopTraits::Goal TeleopTraits::ConvertToGoal(const Request& request) {
    return ToTaskTeleopGoal(request);
}

TeleopTraits::Response TeleopTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    const auto bridge_status =
        static_cast<proto::TeleopStatus>(feedback.status());
    const bool terminal = CheckTeleopTerminalStatus(feedback.status());
    const bool success = feedback.status() != task_proto::TELEOP_STATUS_REJECTED &&
                         feedback.status() != task_proto::TELEOP_STATUS_TIMEOUT;
    std::string message;
    if (feedback.status() == task_proto::TELEOP_STATUS_REJECTED) {
        message = "teleop goal rejected by task";
    } else if (feedback.status() == task_proto::TELEOP_STATUS_TIMEOUT) {
        message = "teleop watchdog timeout";
    }
    return MakeTeleopStatusResponse(last, bridge_status, success, terminal,
                                    message);
}

TeleopTraits::Response TeleopTraits::MakeResponse(const Request& request,
                                                  bool success, bool final,
                                                  const std::string& message) {
    return MakeTeleopStatusResponse(
        request,
        success ? (final ? proto::TELEOP_STATUS_IDLE : proto::TELEOP_STATUS_ACTIVE)
                : proto::TELEOP_STATUS_REJECTED,
        success, final, message);
}

bool TeleopTraits::CheckTerminalStatus(const Feedback& feedback) {
    return CheckTeleopTerminalStatus(feedback.status());
}

bool TeleopTraits::CheckEmitFeedback(const Feedback& feedback,
                                     bool session_active) {
    const auto status = feedback.status();
    if (status == task_proto::TELEOP_STATUS_ACTIVE) {
        return false;
    }
    if (status == task_proto::TELEOP_STATUS_IDLE && !session_active) {
        return false;
    }
    return session_active || CheckTeleopTerminalStatus(status);
}

TeleopStub::TeleopStub(std::shared_ptr<autolink::Node> node,
                       std::shared_ptr<TaskMuxer> muxer)
    : channel_(std::move(node), std::move(muxer)) {}

void TeleopStub::ResetSession() {
    if (channel_.CheckSessionActive()) {
        task_proto::TeleopGoal stop;
        stop.set_command(task_proto::TELEOP_CMD_STOP);
        channel_.WriteGoal(stop);
    }
    channel_.ClearSession(true);
}

bool TeleopStub::HandleCommand(const proto::TeleopCommandRequest& request,
                               StreamCallback stream_callback) {
    if (!stream_callback) {
        AERROR << "TeleopStub: stream callback is null.";
        return false;
    }
    if (!channel_.CheckWriterReady()) {
        stream_callback(MakeTeleopStatusResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "teleop goal writer unavailable"));
        return false;
    }
    if (channel_.CheckEstopActive()) {
        stream_callback(MakeTeleopStatusResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "emergency stop active"));
        return false;
    }

    const auto command = request.command();
    if (command == proto::TELEOP_CMD_UNSPECIFIED) {
        stream_callback(MakeTeleopStatusResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "unspecified teleop command"));
        return false;
    }

    if (command == proto::TELEOP_CMD_START) {
        if (channel_.CheckSessionActive()) {
            stream_callback(MakeTeleopStatusResponse(
                request, proto::TELEOP_STATUS_REJECTED, false, false,
                "teleop session already active"));
            return false;
        }
        if (!channel_.TryAcquireTask(request)) {
            stream_callback(MakeTeleopStatusResponse(
                request, proto::TELEOP_STATUS_REJECTED, false, true,
                "another task is active"));
            return false;
        }
    }

    if (command == proto::TELEOP_CMD_VELOCITY &&
        !channel_.CheckSessionActive()) {
        stream_callback(MakeTeleopStatusResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, false,
            "teleop session not active; send START first"));
        return false;
    }

    channel_.BindStream(request, stream_callback);

    if (!channel_.WriteGoal(TeleopTraits::ConvertToGoal(request))) {
        stream_callback(MakeTeleopStatusResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "failed to publish teleop goal"));
        channel_.ClearSession(true);
        return false;
    }

    const bool handled = DispatchCommands(
        command,
        MakeCommandRule(proto::TELEOP_CMD_START, [&] {
            channel_.SetSessionActive(true);
            stream_callback(MakeTeleopStatusResponse(
                request, proto::TELEOP_STATUS_ACTIVE, true, false));
            return true;
        }),
        MakeCommandRule(proto::TELEOP_CMD_VELOCITY, [&] {
            stream_callback(MakeTeleopStatusResponse(
                request, proto::TELEOP_STATUS_ACTIVE, true, false));
            return true;
        }),
        MakeCommandRule(proto::TELEOP_CMD_STOP, [&] {
            stream_callback(MakeTeleopStatusResponse(
                request, proto::TELEOP_STATUS_IDLE, true, true));
            channel_.ClearSession(true);
            return true;
        }));

    if (handled) {
        return true;
    }
    stream_callback(MakeTeleopStatusResponse(
        request, proto::TELEOP_STATUS_REJECTED, false, true,
        "unsupported command"));
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
