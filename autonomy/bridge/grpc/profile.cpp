/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file profile.cpp
 * @brief Implementation of Fill* / BuildRpcRobotFullInfo System profile helpers.
 */

#include "autonomy/bridge/grpc/profile.hpp"

#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"

#include <chrono>
#include <string>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

constexpr const char* kBridgeVersion = "autonomy.bridge";
constexpr const char* kAutonomyVersion = "autonomy";
constexpr const char* kDefaultModel = "autonomy";

std::string LocalHostname() {
    char host[256] = {};
    if (gethostname(host, sizeof(host) - 1) == 0) {
        return host;
    }
    return {};
}

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

int64_t NowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool CapEnabled(bool has_field, bool value) {
    return !has_field || value;
}

const char* HazardSeverity(::automsgs::rpcs::system::HazardLevel level) {
    using HL = ::automsgs::rpcs::system::HazardLevel;
    switch (level) {
        case HL::HAZARD_LEVEL_WARN:
            return "WARN";
        case HL::HAZARD_LEVEL_ERROR:
            return "ERROR";
        case HL::HAZARD_LEVEL_OK:
            return "OK";
        default:
            return "UNKNOWN";
    }
}

void FillIdentityFromOptions(const proto::RobotIdentityOptions& options,
                             ::automsgs::rpcs::system::RobotIdentity* identity) {
    const std::string host = LocalHostname();
    identity->set_hostname(host);
    identity->set_robot_id(options.robot_id().empty() ? host
                                                     : options.robot_id());
    identity->set_model(options.model().empty() ? kDefaultModel
                                                : options.model());
    identity->set_serial_number(options.serial_number());
    identity->set_firmware_version(options.firmware_version());
    identity->set_software_version(options.software_version().empty()
                                       ? kAutonomyVersion
                                       : options.software_version());
    identity->set_autonomy_version(kAutonomyVersion);
    identity->set_bridge_version(kBridgeVersion);
    identity->set_fleet_id(options.fleet_id());
    identity->set_site_id(options.site_id());
}

template <typename AlertT>
void AddAlert(::google::protobuf::RepeatedPtrField<AlertT>* alerts,
              const std::string& code, const std::string& severity,
              const std::string& message, const std::string& source) {
    auto* alert = alerts->Add();
    alert->set_code(code);
    alert->set_severity(severity);
    alert->set_message(message);
    alert->set_source(source);
}

template <typename AlertT>
void AppendHealthAlerts(
    const ::automsgs::rpcs::system::SystemHealth& health,
    ::google::protobuf::RepeatedPtrField<AlertT>* alerts) {
    using HL = ::automsgs::rpcs::system::HazardLevel;
    if (health.hazard_level() == HL::HAZARD_LEVEL_WARN ||
        health.hazard_level() == HL::HAZARD_LEVEL_ERROR) {
        AddAlert(alerts, "HAZARD", HazardSeverity(health.hazard_level()),
                 health.detail().empty() ? "system hazard"
                                         : health.detail(),
                 "system");
    }
    if (health.mrm_active()) {
        AddAlert(alerts, "MRM_ACTIVE", "WARN", "MRM publishing zero cmd_vel",
                 "system");
    }
    if (health.emergency_stop_latched()) {
        AddAlert(alerts, "ESTOP", "ERROR", "emergency stop latched", "system");
    }
    for (const auto& channel : health.channels()) {
        if (!channel.healthy()) {
            AddAlert(alerts, "CHANNEL_UNHEALTHY", "WARN",
                     "channel unhealthy age_s=" +
                         std::to_string(channel.age_seconds()),
                     channel.channel());
        }
    }
    for (const auto& latency : health.latencies()) {
        if (!latency.healthy()) {
            AddAlert(alerts, "LATENCY_UNHEALTHY", "WARN",
                     "latency unhealthy age_s=" +
                         std::to_string(latency.message_age_seconds()),
                     latency.channel());
        }
    }
}

}  // namespace

void FillRobotIdentity(const proto::RobotIdentityOptions& options,
                       ::automsgs::rpcs::system::RobotIdentity* identity) {
    FillIdentityFromOptions(options, identity);
}

void FillRpcCapabilities(const proto::CapabilitiesOptions& options,
                         ::automsgs::rpcs::system::Capabilities* caps) {
    *caps->mutable_status() = OkStatus();
    caps->set_supports_navigation(CapEnabled(
        options.has_supports_navigation(), options.supports_navigation()));
    caps->set_supports_follow(
        CapEnabled(options.has_supports_follow(), options.supports_follow()));
    caps->set_supports_charge(
        CapEnabled(options.has_supports_charge(), options.supports_charge()));
    caps->set_supports_mapping(CapEnabled(options.has_supports_mapping(),
                                          options.supports_mapping()));
    caps->set_supports_localization(
        CapEnabled(options.has_supports_localization(),
                   options.supports_localization()));
    caps->set_supports_teleop(
        CapEnabled(options.has_supports_teleop(), options.supports_teleop()));
    caps->set_supports_exploration(CapEnabled(
        options.has_supports_exploration(), options.supports_exploration()));
    caps->set_supports_sensor_record(
        CapEnabled(options.has_supports_sensor_record(),
                   options.supports_sensor_record()));
    caps->set_supports_system_monitor(
        CapEnabled(options.has_supports_system_monitor(),
                   options.supports_system_monitor()));
    caps->set_bridge_version(kBridgeVersion);
    caps->set_autonomy_version(kAutonomyVersion);
}

void FillGetInfoResponse(const proto::RobotIdentityOptions& options,
                         ::automsgs::rpcs::system::GetInfoResponse* response) {
    *response->mutable_status() = OkStatus();
    const std::string host = LocalHostname();
    response->set_hostname(host);
    response->set_model(options.model().empty() ? kDefaultModel
                                                : options.model());
    response->set_serial_number(options.serial_number());
    response->set_firmware_version(options.firmware_version());
    response->set_software_version(options.software_version().empty()
                                       ? kAutonomyVersion
                                       : options.software_version());
    response->set_autonomy_version(kAutonomyVersion);
}

::automsgs::rpcs::system::RobotFullInfo BuildRpcRobotFullInfo(
    Context& context) {
    ::automsgs::rpcs::system::RobotFullInfo info;
    *info.mutable_status() = OkStatus();
    FillRobotIdentity(context.identity(), info.mutable_identity());
    auto* state = info.mutable_state();
    *state = context.state_hub().GetSnapshot();

    if (context.muxer().IsEstop()) {
        info.set_system_state(::automsgs::rpcs::system::SYSTEM_STATE_ESTOP);
    } else if (context.muxer().HasActive()) {
        info.set_system_state(::automsgs::rpcs::system::SYSTEM_STATE_BUSY);
    } else {
        info.set_system_state(::automsgs::rpcs::system::SYSTEM_STATE_IDLE);
    }

    const auto task = context.muxer().GetSnapshot();
    auto* goal = info.mutable_active_goal();
    *goal->mutable_status() = OkStatus();
    goal->set_kind(ToGoalKind(task.type));
    goal->set_goal_id(task.cmd_id);

    const auto health = context.system_monitor().GetHealth(true);
    *info.mutable_health() = health;
    AppendHealthAlerts(health, info.mutable_alerts());
    if (context.muxer().IsEstop() && !health.emergency_stop_latched()) {
        AddAlert(info.mutable_alerts(), "ESTOP", "ERROR",
                 "muxer emergency stop active", "muxer");
    }

    FillRpcCapabilities(context.capabilities(), info.mutable_capabilities());

    const std::string map_name = context.mapping().GetCurrentMapName();
    if (state->map_name().empty() && !map_name.empty()) {
        state->set_map_name(map_name);
    }

    info.set_robot_time_ns(NowNs());
    return info;
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
