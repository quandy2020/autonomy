/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/teleop_stub.hpp"

#include <optional>

#include "autolink/common/log.hpp"
#include "autonomy/bridge/grpc/clients/command_dispatch.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace teleop_rpc = ::automsgs::rpcs::teleop;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool CheckTeleopTerminalStatus(task_proto::TeleopStatus status) {
    return status == task_proto::TELEOP_STATUS_TIMEOUT ||
           status == task_proto::TELEOP_STATUS_REJECTED ||
           status == task_proto::TELEOP_STATUS_IDLE;
}

teleop_rpc::TeleopState ToTeleopState(task_proto::TeleopStatus status) {
    switch (status) {
        case task_proto::TELEOP_STATUS_ACTIVE:
            return teleop_rpc::TELEOP_STATE_ACTIVE;
        case task_proto::TELEOP_STATUS_IDLE:
            return teleop_rpc::TELEOP_STATE_IDLE;
        case task_proto::TELEOP_STATUS_TIMEOUT:
            return teleop_rpc::TELEOP_STATE_TIMEOUT;
        case task_proto::TELEOP_STATUS_REJECTED:
            return teleop_rpc::TELEOP_STATE_REJECTED;
        default:
            return teleop_rpc::TELEOP_STATE_UNKNOWN;
    }
}

teleop_rpc::TeleopResponse MakeTeleopFrame(const std::string& goal_id,
                                           teleop_rpc::TeleopState state,
                                           bool ok,
                                           const std::string& detail = "") {
    teleop_rpc::TeleopResponse response;
    response.set_goal_id(goal_id);
    response.set_state(state);
    response.set_detail(detail);
    *response.mutable_status() =
        ok ? OkStatus(detail)
           : ErrorStatus(StatusCode::TELEOP_BUSY, detail);
    return response;
}

}  // namespace

TeleopTraits::Goal TeleopTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    switch (request.command()) {
        case teleop_rpc::VELOCITY_COMMAND_START:
            goal.set_command(task_proto::TELEOP_CMD_START);
            break;
        case teleop_rpc::VELOCITY_COMMAND_STOP:
            goal.set_command(task_proto::TELEOP_CMD_STOP);
            break;
        case teleop_rpc::VELOCITY_COMMAND_TWIST:
            goal.set_command(task_proto::TELEOP_CMD_VELOCITY);
            break;
        default:
            goal.set_command(task_proto::TELEOP_CMD_UNSPECIFIED);
            break;
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_TELEOP);
    if (request.has_twist()) {
        *goal.mutable_velocity() = request.twist();
    }
    if (request.has_options()) {
        if (request.options().has_maximum_linear_speed()) {
            goal.set_max_linear_speed(request.options().maximum_linear_speed());
        }
        if (request.options().has_maximum_angular_speed()) {
            goal.set_max_angular_speed(
                request.options().maximum_angular_speed());
        }
        if (request.options().has_watchdog_timeout_seconds()) {
            goal.set_watchdog_timeout_sec(
                request.options().watchdog_timeout_seconds());
        }
        goal.set_disable_collision_checks(
            request.options().disable_collision_checks());
    }
    return goal;
}

TeleopTraits::Response TeleopTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    const bool ok = feedback.status() != task_proto::TELEOP_STATUS_REJECTED &&
                    feedback.status() != task_proto::TELEOP_STATUS_TIMEOUT;
    std::string detail;
    if (feedback.status() == task_proto::TELEOP_STATUS_REJECTED) {
        detail = "teleop goal rejected by task";
    } else if (feedback.status() == task_proto::TELEOP_STATUS_TIMEOUT) {
        detail = "teleop watchdog timeout";
    }
    return MakeTeleopFrame(last.goal_id(), ToTeleopState(feedback.status()), ok,
                           detail);
}

TeleopTraits::Response TeleopTraits::MakeResponse(const Request& request,
                                                  bool success, bool final,
                                                  const std::string& message) {
    return MakeTeleopFrame(
        request.goal_id(),
        success ? (final ? teleop_rpc::TELEOP_STATE_IDLE
                         : teleop_rpc::TELEOP_STATE_ACTIVE)
                : teleop_rpc::TELEOP_STATE_REJECTED,
        success, message);
}

bool TeleopTraits::IsTerminal(const Feedback& feedback) {
    return CheckTeleopTerminalStatus(feedback.status());
}

bool TeleopGoalChannelTraits::ShouldEmit(const Feedback& feedback,
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

std::optional<std::string> TeleopGoalChannelTraits::RejectReason(
    const Request& request, bool session_active) {
    const auto command = request.command();
    if (command == teleop_rpc::VELOCITY_COMMAND_UNKNOWN) {
        return std::string("unspecified teleop command");
    }
    if (command == teleop_rpc::VELOCITY_COMMAND_START && session_active) {
        return std::string("teleop session already active");
    }
    if (command == teleop_rpc::VELOCITY_COMMAND_TWIST && !session_active) {
        return std::string("teleop session not active; send START first");
    }
    return std::nullopt;
}

TeleopStub::TeleopStub(std::shared_ptr<autolink::Node> node,
                       TaskMuxer::SharedPtr muxer,
                       WorkScheduler* scheduler,
                       CommandIdempotencyCache* idempotency)
    : GoalChannelCommandStub(node, muxer),
      relative_(teleop::TeleopRelativeBackend::make_unique(
          node, muxer, scheduler, idempotency)) {
    relative_->SetVelocityBusyCheck([this] { return channel_.IsActive(); });
}

bool TeleopStub::CancelGoal(const std::string& goal_id) {
    relative_->CancelGoal(goal_id);
    GoalChannelCommandStub::CancelGoal(goal_id);
    return true;
}

teleop_rpc::TeleopResponse TeleopStub::GetSnapshot() const {
    if (relative_->IsBusy()) {
        return relative_->GetSnapshot();
    }
    return MakeTeleopFrame(
        {},
        channel_.IsActive() ? teleop_rpc::TELEOP_STATE_ACTIVE
                            : teleop_rpc::TELEOP_STATE_IDLE,
        true);
}

bool TeleopStub::HandleVelocity(const teleop_rpc::VelocityRequest& request,
                                StreamCallback stream_callback) {
    if (!stream_callback) {
        AERROR << "TeleopStub: stream callback is null.";
        return false;
    }
    if (relative_->IsBusy()) {
        stream_callback(MakeTeleopFrame(
            request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
            "relative teleop goal is active"));
        return false;
    }
    const auto command = request.command();
    const bool acquire = command == teleop_rpc::VELOCITY_COMMAND_START;
    return channel_.Dispatch(
        request, std::move(stream_callback), acquire,
        [&](const StreamCallback& emit) {
            const bool handled = DispatchCommands(
                command,
                MakeCommandRule(teleop_rpc::VELOCITY_COMMAND_START, [&] {
                    channel_.SetSessionActive(true);
                    emit(MakeTeleopFrame(request.goal_id(),
                                         teleop_rpc::TELEOP_STATE_ACTIVE, true));
                    return true;
                }),
                MakeCommandRule(teleop_rpc::VELOCITY_COMMAND_TWIST, [&] {
                    emit(MakeTeleopFrame(request.goal_id(),
                                         teleop_rpc::TELEOP_STATE_ACTIVE, true));
                    return true;
                }),
                MakeCommandRule(teleop_rpc::VELOCITY_COMMAND_STOP, [&] {
                    emit(MakeTeleopFrame(request.goal_id(),
                                         teleop_rpc::TELEOP_STATE_IDLE, true));
                    channel_.ClearSession(true);
                    return true;
                }));
            if (!handled) {
                emit(MakeTeleopFrame(request.goal_id(),
                                     teleop_rpc::TELEOP_STATE_REJECTED, false,
                                     "unsupported command"));
            }
            return handled;
        });
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
