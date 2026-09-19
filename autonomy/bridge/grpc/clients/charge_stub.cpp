/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/charge_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>
#include "autolink/common/log.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

namespace {

namespace task_proto = ::autonomy::task::proto;
namespace charge_rpc = ::automsgs::rpcs::charge;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool CheckChargingTerminalStatus(task_proto::DockStatus status) {
    return status == task_proto::DOCK_STATUS_SUCCEEDED ||
           status == task_proto::DOCK_STATUS_FAILED ||
           status == task_proto::DOCK_STATUS_CANCELED;
}

charge_rpc::ChargeState ToChargeState(task_proto::DockStatus status) {
    switch (status) {
        case task_proto::DOCK_STATUS_IDLE:
            return charge_rpc::CHARGE_STATE_IDLE;
        case task_proto::DOCK_STATUS_SEARCHING:
        case task_proto::DOCK_STATUS_APPROACHING:
        case task_proto::DOCK_STATUS_DOCKING:
            return charge_rpc::CHARGE_STATE_RETURNING;
        case task_proto::DOCK_STATUS_CHARGING:
            return charge_rpc::CHARGE_STATE_CHARGING;
        case task_proto::DOCK_STATUS_UNDOCKING:
            return charge_rpc::CHARGE_STATE_LEAVING;
        case task_proto::DOCK_STATUS_SUCCEEDED:
            return charge_rpc::CHARGE_STATE_FULL;
        case task_proto::DOCK_STATUS_FAILED:
            return charge_rpc::CHARGE_STATE_FAILED;
        case task_proto::DOCK_STATUS_CANCELED:
            return charge_rpc::CHARGE_STATE_CANCELLED;
        default:
            return charge_rpc::CHARGE_STATE_UNKNOWN;
    }
}

charge_rpc::ChargeResponse MakeChargeFrame(const std::string& goal_id,
                                           const std::string& station_id,
                                           charge_rpc::ChargeState state,
                                           bool ok, bool active,
                                           const std::string& message = "") {
    charge_rpc::ChargeResponse response;
    response.set_goal_id(goal_id);
    response.set_station_id(station_id);
    response.set_state(state);
    response.set_active(active);
    response.set_message(message);
    *response.mutable_status() =
        ok ? OkStatus(message)
           : ErrorStatus(StatusCode::CHARGING_BUSY, message);
    return response;
}

}  // namespace

ChargeTraits::Goal ChargeTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    goal.set_command(task_proto::DOCK_CMD_START);
    if (!request.station_id().empty()) {
        goal.set_dock_station_id(request.station_id());
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK);
    return goal;
}

ChargeTraits::Response ChargeTraits::ConvertFromFeedback(const Feedback& feedback,
                                                     const Request& last) {
    Response response = MakeChargeFrame(
        last.goal_id(), feedback.dock_station_id().empty()
                            ? last.station_id()
                            : feedback.dock_station_id(),
        ToChargeState(feedback.status()),
        !CheckChargingTerminalStatus(feedback.status()) ||
            feedback.status() == task_proto::DOCK_STATUS_SUCCEEDED,
        !CheckChargingTerminalStatus(feedback.status()));
    response.set_battery_pct(feedback.battery_percent());
    return response;
}

ChargeTraits::Response ChargeTraits::MakeResponse(const Request& request,
                                              bool success, bool final,
                                              const std::string& message) {
    return MakeChargeFrame(
        request.goal_id(), request.station_id(),
        success ? (final ? charge_rpc::CHARGE_STATE_CANCELLED
                         : charge_rpc::CHARGE_STATE_RETURNING)
                : charge_rpc::CHARGE_STATE_FAILED,
        success, success && !final, message);
}

bool ChargeTraits::IsTerminal(const Feedback& feedback) {
    return CheckChargingTerminalStatus(feedback.status());
}

bool ChargeStub::HandleLeave(const charge_rpc::LeaveRequest& request,
                           StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    charge_rpc::ReturnRequest session;
    session.set_goal_id(request.goal_id());
    if (request.has_header()) {
        *session.mutable_header() = request.header();
    }
    auto reject = [&](const std::string& message) {
        stream_callback(MakeChargeFrame(request.goal_id(), "",
                                        charge_rpc::CHARGE_STATE_FAILED, false,
                                        false, message));
    };
    if (!channel_.CheckWriterReady()) {
        reject("goal writer unavailable");
        return false;
    }
    if (channel_.IsEstop()) {
        reject("emergency stop active");
        return false;
    }
    if (!channel_.TryAcquireTask(session)) {
        reject("another task is already active");
        return false;
    }
    channel_.BindStream(session, stream_callback);
    task_proto::ChargingGoal undock;
    undock.set_command(task_proto::DOCK_CMD_UNDOCK);
    SetTaskHeader(&undock, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK);
    if (!channel_.WriteGoal(undock)) {
        channel_.ClearSession(true);
        reject("failed to publish goal");
        return false;
    }
    channel_.SetSessionActive(true);
    stream_callback(MakeChargeFrame(request.goal_id(), "",
                                    charge_rpc::CHARGE_STATE_LEAVING, true, true,
                                    ""));
    return true;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
