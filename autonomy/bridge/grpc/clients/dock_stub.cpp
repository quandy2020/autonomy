/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/dock_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;

proto::TaskStatus ResolveDockTaskStatus(proto::DockStatus status) {
    switch (status) {
        case proto::DOCK_STATUS_SEARCHING:
        case proto::DOCK_STATUS_APPROACHING:
        case proto::DOCK_STATUS_DOCKING:
        case proto::DOCK_STATUS_CHARGING:
        case proto::DOCK_STATUS_UNDOCKING:
            return proto::TASK_STATUS_RUNNING;
        case proto::DOCK_STATUS_SUCCEEDED:
            return proto::TASK_STATUS_SUCCEEDED;
        case proto::DOCK_STATUS_FAILED:
            return proto::TASK_STATUS_FAILED;
        case proto::DOCK_STATUS_CANCELED:
            return proto::TASK_STATUS_CANCELED;
        default:
            return proto::TASK_STATUS_IDLE;
    }
}

bool CheckDockTerminalStatus(task_proto::DockStatus status) {
    return status == task_proto::DOCK_STATUS_SUCCEEDED ||
           status == task_proto::DOCK_STATUS_FAILED ||
           status == task_proto::DOCK_STATUS_CANCELED;
}

DockTraits::Response MakeDockStatusResponse(
    const DockTraits::Request& request, proto::DockStatus status, bool success,
    bool final, const std::string& message = "") {
    DockTraits::Response response;
    response.set_status(status);
    FillCommandAck(response, DockTraits::kTaskType, request, success, final,
                   ResolveDockTaskStatus(status), message);
    return response;
}

}  // namespace

DockTraits::Goal DockTraits::ConvertToGoal(const Request& request) {
    return ToTaskChargingGoal(request);
}

DockTraits::Response DockTraits::ConvertFromFeedback(const Feedback& feedback,
                                                     const Request& last) {
    Response response;
    response.set_status(static_cast<proto::DockStatus>(feedback.status()));
    response.set_battery_percent(feedback.battery_percent());
    response.set_dock_station_id(feedback.dock_station_id());
    FillCommandAck(response, kTaskType, last,
                   !CheckDockTerminalStatus(feedback.status()) ||
                       feedback.status() == task_proto::DOCK_STATUS_SUCCEEDED,
                   CheckDockTerminalStatus(feedback.status()),
                   ResolveDockTaskStatus(response.status()));
    return response;
}

DockTraits::Response DockTraits::MakeResponse(const Request& request,
                                              bool success, bool final,
                                              const std::string& message) {
    return MakeDockStatusResponse(
        request,
        success ? (final ? proto::DOCK_STATUS_CANCELED
                         : proto::DOCK_STATUS_SEARCHING)
                : proto::DOCK_STATUS_FAILED,
        success, final, message);
}

bool DockTraits::CheckTerminalStatus(const Feedback& feedback) {
    return CheckDockTerminalStatus(feedback.status());
}

DockStub::DockStub(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<TaskMuxer> muxer)
    : channel_(std::move(node), std::move(muxer)) {}

void DockStub::CancelActiveSession() {
    task_proto::ChargingGoal cancel;
    cancel.set_command(task_proto::DOCK_CMD_CANCEL);
    channel_.WriteGoal(cancel);
    channel_.ClearSession(true);
}

bool DockStub::HandleCommand(const proto::DockCommandRequest& request,
                             StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    if (!channel_.CheckWriterReady()) {
        stream_callback(MakeDockStatusResponse(
            request, proto::DOCK_STATUS_FAILED, false, true,
            "dock goal writer unavailable"));
        return false;
    }
    if (channel_.CheckEstopActive()) {
        stream_callback(MakeDockStatusResponse(
            request, proto::DOCK_STATUS_FAILED, false, true,
            "emergency stop active"));
        return false;
    }

    const auto command = request.command();
    if (command == proto::DOCK_CMD_START || command == proto::DOCK_CMD_UNDOCK) {
        if (!channel_.TryAcquireTask(request)) {
            stream_callback(MakeDockStatusResponse(
                request, proto::DOCK_STATUS_FAILED, false, true,
                "another task is active"));
            return false;
        }
    }

    channel_.BindStream(request, stream_callback);

    if (!channel_.WriteGoal(DockTraits::ConvertToGoal(request))) {
        stream_callback(MakeDockStatusResponse(
            request, proto::DOCK_STATUS_FAILED, false, true,
            "failed to publish dock goal"));
        channel_.ClearSession(true);
        return false;
    }

    const bool handled = DispatchCommands(
        command,
        MakeCommandRule(proto::DOCK_CMD_START, [&] {
            channel_.SetSessionActive(true);
            stream_callback(MakeDockStatusResponse(
                request, proto::DOCK_STATUS_SEARCHING, true, false));
            return true;
        }),
        MakeCommandRule(proto::DOCK_CMD_UNDOCK, [&] {
            channel_.SetSessionActive(true);
            stream_callback(MakeDockStatusResponse(
                request, proto::DOCK_STATUS_UNDOCKING, true, false));
            return true;
        }),
        MakeCommandRule(proto::DOCK_CMD_PAUSE, [&] {
            stream_callback(MakeDockStatusResponse(
                request, proto::DOCK_STATUS_CHARGING, true, false, "paused"));
            return true;
        }),
        MakeCommandRule(proto::DOCK_CMD_RESUME, [&] {
            channel_.SetSessionActive(true);
            stream_callback(MakeDockStatusResponse(
                request, proto::DOCK_STATUS_DOCKING, true, false));
            return true;
        }),
        MakeCommandRules(
            [&] {
                stream_callback(MakeDockStatusResponse(
                    request, proto::DOCK_STATUS_CANCELED, true, true));
                channel_.ClearSession(true);
                return true;
            },
            proto::DOCK_CMD_STOP, proto::DOCK_CMD_CANCEL));

    if (handled) {
        return true;
    }
    stream_callback(MakeDockStatusResponse(request, proto::DOCK_STATUS_FAILED,
                                           false, true, "unsupported command"));
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
