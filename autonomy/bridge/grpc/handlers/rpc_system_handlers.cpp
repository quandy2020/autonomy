/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_system_handlers.cpp
 * @brief Implementation of SystemService RpcHandler OnRequest bodies.
 */

#include "autonomy/bridge/grpc/handlers/rpc_system_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/util.hpp"
#include "autonomy/bridge/grpc/profile.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/system/logging/event_bundler.hpp"
#include "autonomy/system/ota/ota_agent.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {
namespace {

::automsgs::rpcs::system::GoalKind ToGoalKind(TaskType type) {
    switch (type) {
        case TASK_TYPE_NAVIGATION:
            return ::automsgs::rpcs::system::GOAL_KIND_NAVIGATION;
        case TASK_TYPE_FOLLOW:
            return ::automsgs::rpcs::system::GOAL_KIND_FOLLOW;
        case TASK_TYPE_TELEOP:
            return ::automsgs::rpcs::system::GOAL_KIND_TELEOP;
        case TASK_TYPE_EXPLORATION:
            return ::automsgs::rpcs::system::GOAL_KIND_EXPLORATION;
        case TASK_TYPE_DOCK:
            return ::automsgs::rpcs::system::GOAL_KIND_CHARGE;
        case TASK_TYPE_MAP:
            return ::automsgs::rpcs::system::GOAL_KIND_MAPPING;
        case TASK_TYPE_VOICE:
            return ::automsgs::rpcs::system::GOAL_KIND_VOICE;
        default:
            return ::automsgs::rpcs::system::GOAL_KIND_UNKNOWN;
    }
}

}  // namespace

void RpcSystemHeartbeatHandler::OnRequest(
    const ::automsgs::rpcs::system::HeartbeatRequest& request) {
    ::automsgs::rpcs::system::HeartbeatResponse response;
    *response.mutable_status() = OkStatus();
    response.set_sequence(request.sequence());
    response.set_robot_time_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    ReplyUnary(this, std::move(response));
}

void RpcSystemGetInfoHandler::OnRequest(
    const ::automsgs::rpcs::system::GetInfoRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetInfoResponse response;
        FillGetInfoResponse(context->identity(), &response);
        return response;
    });
}

void RpcSystemGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::system::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetStatusResponse response;
        *response.mutable_status() = OkStatus();
        if (context->muxer().IsEstop()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_ESTOP);
        } else if (context->muxer().HasActive()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_BUSY);
        } else {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_IDLE);
        }
        const auto snapshot = context->state_hub().GetSnapshot();
        const float percent = snapshot.battery_percent();
        response.mutable_battery()->set_percentage(
            percent > 1.f ? percent / 100.f : percent);
        *response.mutable_health() = context->system_monitor().GetHealth(false);
        return response;
    });
}

void RpcSystemGetHealthHandler::OnRequest(
    const ::automsgs::rpcs::system::GetHealthRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetHealthResponse response;
        *response.mutable_status() = OkStatus();
        *response.mutable_health() = context->system_monitor().GetHealth(true);
        return response;
    });
}

void RpcSystemGetRobotFullInfoHandler::OnRequest(
    const ::automsgs::rpcs::system::GetRobotFullInfoRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return BuildRpcRobotFullInfo(*context);
    });
}

void RpcSystemGetCapabilitiesHandler::OnRequest(
    const ::automsgs::rpcs::system::GetCapabilitiesRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::Capabilities response;
        FillRpcCapabilities(context->capabilities(), &response);
        return response;
    });
}

void RpcSystemEmergencyStopHandler::OnRequest(
    const ::automsgs::rpcs::system::EmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(true);
        return OkStatus("estop engaged");
    });
}

void RpcSystemClearEmergencyStopHandler::OnRequest(
    const ::automsgs::rpcs::system::ClearEmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(false);
        return OkStatus("estop cleared");
    });
}

void RpcSystemCancelAllGoalsHandler::OnRequest(
    const ::automsgs::rpcs::system::CancelAllGoalsRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->CancelAllTasks();
        return OkStatus("cancelled");
    });
}

void RpcSystemGetActiveGoalHandler::OnRequest(
    const ::automsgs::rpcs::system::GetActiveGoalRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::ActiveGoal response;
        *response.mutable_status() = OkStatus();
        const auto snapshot = context->muxer().GetSnapshot();
        response.set_kind(ToGoalKind(snapshot.type));
        response.set_goal_id(snapshot.cmd_id);
        return response;
    });
}

void RpcSystemRestartModuleHandler::OnRequest(
    const ::automsgs::rpcs::system::RestartModuleRequest& request) {
    ::automsgs::rpcs::system::RestartModuleResponse response;
    if (request.module_name().empty()) {
        *response.mutable_status() = ErrorStatus(
            ::automsgs::msgs::status_msgs::INVALID_ARGUMENT,
            "module_name required");
        ReplyUnary(this, std::move(response));
        return;
    }
    namespace fs = std::filesystem;
    const char* xdg = std::getenv("XDG_RUNTIME_DIR");
    fs::path dir =
        xdg && xdg[0] ? fs::path(xdg) / "autonomy" : fs::path("/tmp/autonomy");
    std::error_code ec;
    fs::create_directories(dir, ec);
    std::ofstream out(dir / "restart_module.request");
    out << request.module_name() << "\n" << request.reason() << "\n";
    *response.mutable_status() = OkStatus("restart requested");
    response.set_detail((dir / "restart_module.request").string());
    ReplyUnary(this, std::move(response));
}

void RpcSystemGetProcessStatusHandler::OnRequest(
    const ::automsgs::rpcs::system::GetProcessStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetProcessStatusResponse response;
        *response.mutable_status() = OkStatus();
        const auto health = context->system_monitor().GetHealth(true);
        for (const auto& p : health.processes()) {
            *response.add_processes() = p;
        }
        return response;
    });
}

void RpcSystemGetEventBundleHandler::OnRequest(
    const ::automsgs::rpcs::system::GetEventBundleRequest& request) {
    ::automsgs::rpcs::system::GetEventBundleResponse response;
    autonomy::system::logging::EventBundler bundler;
    std::string path;
    if (request.event_id().empty()) {
        path = bundler.LatestEventPath();
    } else {
        path = bundler.ResolveEventPath(request.event_id());
    }
    if (path.empty() || !std::filesystem::exists(path)) {
        *response.mutable_status() = ErrorStatus(
            ::automsgs::msgs::status_msgs::NOT_FOUND, "no event bundle");
        ReplyUnary(this, std::move(response));
        return;
    }
    *response.mutable_status() = OkStatus();
    response.set_event_id(std::filesystem::path(path).filename().string());
    response.set_path(path);
    ReplyUnary(this, std::move(response));
}

void RpcSystemStartOtaHandler::OnRequest(
    const ::automsgs::rpcs::system::StartOtaRequest& request) {
    ReplyUnary(this,
               autonomy::system::ota::OtaAgent::Shared().Start(request));
}

void RpcSystemGetOtaStatusHandler::OnRequest(
    const ::automsgs::rpcs::system::GetOtaStatusRequest&) {
    ReplyUnary(this, autonomy::system::ota::OtaAgent::Shared().Status());
}

void RpcSystemAbortOtaHandler::OnRequest(
    const ::automsgs::rpcs::system::AbortOtaRequest& request) {
    ReplyUnary(this,
               autonomy::system::ota::OtaAgent::Shared().Abort(request.reason()));
}

void RpcSystemApplyConfigPackageHandler::OnRequest(
    const ::automsgs::rpcs::system::ApplyConfigPackageRequest& request) {
    ::automsgs::rpcs::system::ApplyConfigPackageResponse response;
    if (request.package_uri().empty()) {
        *response.mutable_status() = ErrorStatus(
            ::automsgs::msgs::status_msgs::INVALID_ARGUMENT,
            "package_uri required");
        ReplyUnary(this, std::move(response));
        return;
    }
    if (request.dry_run()) {
        *response.mutable_status() = OkStatus("dry_run ok");
        response.set_detail(request.package_uri());
        ReplyUnary(this, std::move(response));
        return;
    }
    namespace fs = std::filesystem;
    const fs::path src(request.package_uri());
    if (!fs::exists(src)) {
        *response.mutable_status() = ErrorStatus(
            ::automsgs::msgs::status_msgs::NOT_FOUND, "package missing");
        ReplyUnary(this, std::move(response));
        return;
    }
    const char* home = std::getenv("HOME");
    fs::path dest = home && home[0] ? fs::path(home) / ".autonomy/config_drop"
                                    : fs::path("/tmp/autonomy/config_drop");
    std::error_code ec;
    fs::create_directories(dest, ec);
    if (fs::is_directory(src)) {
        fs::copy(src, dest / src.filename(),
                 fs::copy_options::recursive |
                     fs::copy_options::overwrite_existing,
                 ec);
    } else {
        fs::copy_file(src, dest / src.filename(),
                      fs::copy_options::overwrite_existing, ec);
    }
    if (ec) {
        *response.mutable_status() =
            ErrorStatus(::automsgs::msgs::status_msgs::INTERNAL, ec.message());
        ReplyUnary(this, std::move(response));
        return;
    }
    *response.mutable_status() = OkStatus("staged");
    response.set_detail(dest.string());
    ReplyUnary(this, std::move(response));
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
