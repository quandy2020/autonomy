/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/follow_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;

proto::TaskStatus ResolveFollowTaskStatus(proto::FollowStatus status) {
    switch (status) {
        case proto::FOLLOW_STATUS_FOLLOWING:
        case proto::FOLLOW_STATUS_TARGET_LOST:
            return proto::TASK_STATUS_RUNNING;
        case proto::FOLLOW_STATUS_PAUSED:
            return proto::TASK_STATUS_PAUSED;
        case proto::FOLLOW_STATUS_SUCCEEDED:
            return proto::TASK_STATUS_SUCCEEDED;
        case proto::FOLLOW_STATUS_FAILED:
            return proto::TASK_STATUS_FAILED;
        case proto::FOLLOW_STATUS_CANCELED:
            return proto::TASK_STATUS_CANCELED;
        default:
            return proto::TASK_STATUS_IDLE;
    }
}

bool CheckTrackerTerminalStatus(task_proto::TrackerStatus status) {
    return status == task_proto::TRACKER_STATUS_SUCCEEDED ||
           status == task_proto::TRACKER_STATUS_FAILED ||
           status == task_proto::TRACKER_STATUS_CANCELED;
}

}  // namespace

FollowTraits::Goal FollowTraits::ConvertToGoal(const Request& request) {
    return ToTaskTrackerGoal(request);
}

FollowTraits::Response FollowTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_status(static_cast<proto::FollowStatus>(feedback.status()));
    response.set_distance_to_target(feedback.distance_to_target());
    if (feedback.has_target_pose()) {
        *response.mutable_target_pose() = feedback.target_pose();
    }
    FillCommandAck(response, kTaskType, last,
                   !CheckTrackerTerminalStatus(feedback.status()) ||
                       feedback.status() == task_proto::TRACKER_STATUS_SUCCEEDED,
                   CheckTrackerTerminalStatus(feedback.status()),
                   ResolveFollowTaskStatus(response.status()));
    return response;
}

FollowTraits::Response FollowTraits::MakeResponse(const Request& request,
                                                  bool success, bool final,
                                                  const std::string& message) {
    Response response;
    response.set_status(success ? (final ? proto::FOLLOW_STATUS_CANCELED
                                         : proto::FOLLOW_STATUS_FOLLOWING)
                                : proto::FOLLOW_STATUS_FAILED);
    FillCommandAck(response, kTaskType, request, success, final,
                   ResolveFollowTaskStatus(response.status()), message);
    return response;
}

bool FollowTraits::CheckTerminalStatus(const Feedback& feedback) {
    return CheckTrackerTerminalStatus(feedback.status());
}

FollowStub::FollowStub(std::shared_ptr<autolink::Node> node,
                       std::shared_ptr<TaskMuxer> muxer)
    : channel_(std::move(node), std::move(muxer)) {}

void FollowStub::CancelActiveSession() {
    task_proto::TrackerGoal cancel;
    cancel.set_command(task_proto::TRACKER_CMD_CANCEL);
    channel_.WriteGoal(cancel);
    channel_.ClearSession(true);
}

bool FollowStub::HandleCommand(const proto::FollowCommandRequest& request,
                               StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    if (!channel_.CheckWriterReady()) {
        stream_callback(FollowTraits::MakeResponse(
            request, false, true, "follow goal writer unavailable"));
        return false;
    }
    if (channel_.CheckEstopActive()) {
        stream_callback(FollowTraits::MakeResponse(
            request, false, true, "emergency stop active"));
        return false;
    }

    const auto command = request.command();
    if (command == proto::FOLLOW_CMD_START ||
        command == proto::FOLLOW_CMD_UPDATE_TARGET) {
        if (!channel_.TryAcquireTask(request)) {
            stream_callback(FollowTraits::MakeResponse(
                request, false, true, "another task is active"));
            return false;
        }
    }

    channel_.BindStream(request, stream_callback);

    if (!channel_.WriteGoal(FollowTraits::ConvertToGoal(request))) {
        stream_callback(FollowTraits::MakeResponse(
            request, false, true, "failed to publish follow goal"));
        channel_.ClearSession(true);
        return false;
    }

    const bool handled = DispatchCommands(
        command,
        MakeCommandRules(
            [&] {
                channel_.SetSessionActive(true);
                stream_callback(FollowTraits::MakeResponse(
                    request, true, false, ""));
                return true;
            },
            proto::FOLLOW_CMD_START, proto::FOLLOW_CMD_UPDATE_TARGET,
            proto::FOLLOW_CMD_RESUME),
        MakeCommandRule(proto::FOLLOW_CMD_PAUSE, [&] {
            proto::FollowCommandResponse paused;
            paused.set_status(proto::FOLLOW_STATUS_PAUSED);
            FillCommandAck(paused, proto::TASK_TYPE_FOLLOW, request, true,
                           false, proto::TASK_STATUS_PAUSED);
            stream_callback(paused);
            return true;
        }),
        MakeCommandRules(
            [&] {
                stream_callback(FollowTraits::MakeResponse(
                    request, true, true, ""));
                channel_.ClearSession(true);
                return true;
            },
            proto::FOLLOW_CMD_STOP, proto::FOLLOW_CMD_CANCEL));

    if (handled) {
        return true;
    }
    stream_callback(FollowTraits::MakeResponse(request, false, true,
                                               "unsupported command"));
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
