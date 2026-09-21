/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/health_snapshot_store.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace system {
namespace monitor {
namespace {

namespace fs = std::filesystem;

std::string RuntimeRoot() {
    if (const char* xdg = std::getenv("XDG_RUNTIME_DIR");
        xdg != nullptr && xdg[0] != '\0') {
        return std::string(xdg) + "/autonomy";
    }
    if (const char* home = std::getenv("HOME");
        home != nullptr && home[0] != '\0') {
        return std::string(home) + "/.autonomy/runtime";
    }
    return "/tmp/autonomy";
}

::automsgs::rpcs::system::HazardLevel ToHazardLevel(HazardLevel level) {
    switch (level) {
        case HazardLevel::kWarn:
            return ::automsgs::rpcs::system::HAZARD_LEVEL_WARN;
        case HazardLevel::kError:
            return ::automsgs::rpcs::system::HAZARD_LEVEL_ERROR;
        case HazardLevel::kOk:
        default:
            return ::automsgs::rpcs::system::HAZARD_LEVEL_OK;
    }
}

}  // namespace

std::string HealthSnapshotStore::DefaultPath() {
    if (const char* override_path = std::getenv("AUTONOMY_HEALTH_SNAPSHOT_PATH");
        override_path != nullptr && override_path[0] != '\0') {
        return override_path;
    }
    return RuntimeRoot() + "/health_snapshot.pb";
}

HealthSnapshotStore::HealthSnapshotStore(std::string path)
    : path_(std::move(path)) {}

::automsgs::rpcs::system::SystemHealth HealthSnapshotStore::ToProto(
    const SystemHealthSnapshot& snapshot) const {
    ::automsgs::rpcs::system::SystemHealth health;
    health.set_hazard_level(ToHazardLevel(snapshot.hazard_level));
    health.set_mrm_active(snapshot.mrm_active);
    health.set_emergency_stop_latched(snapshot.emergency_stop_latched);
    if (!snapshot.detail.empty()) {
        health.set_detail(snapshot.detail);
    }

    auto* host = health.mutable_host();
    if (snapshot.cpu_usage_percent >= 0.f) {
        host->set_cpu_usage_percent(snapshot.cpu_usage_percent);
    }
    if (snapshot.memory_usage_percent >= 0.f) {
        host->set_memory_usage_percent(snapshot.memory_usage_percent);
    }
    if (snapshot.disk_usage_percent >= 0.f) {
        host->set_disk_usage_percent(snapshot.disk_usage_percent);
    }
    if (snapshot.load_average_1m >= 0.f) {
        host->set_load_average_1m(snapshot.load_average_1m);
    }
    host->set_ntp_offset_seconds(snapshot.ntp_offset_seconds);

    for (const auto& channel : snapshot.channels) {
        auto* info = health.add_channels();
        info->set_channel(channel.channel);
        info->set_ever_received(channel.ever_received);
        info->set_healthy(channel.healthy);
        info->set_age_seconds(static_cast<float>(channel.age_sec));
        info->set_rate_hz(static_cast<float>(channel.rate_hz));
    }
    for (const auto& latency : snapshot.latencies) {
        auto* info = health.add_latencies();
        info->set_channel(latency.channel);
        info->set_ever_received(latency.ever_received);
        info->set_healthy(latency.healthy);
        info->set_message_age_seconds(
            static_cast<float>(latency.message_age_sec));
    }
    for (const auto& proc : snapshot.processes) {
        auto* info = health.add_processes();
        info->set_name(proc.name);
        info->set_match(proc.match);
        info->set_alive(proc.alive);
        info->set_pid(proc.pid);
        info->set_restart_hint(proc.restart_hint);
    }
    return health;
}

bool HealthSnapshotStore::Write(const SystemHealthSnapshot& snapshot) const {
    try {
        fs::create_directories(fs::path(path_).parent_path());
    } catch (...) {
        AWARN << "HealthSnapshotStore: cannot create parent for " << path_;
        return false;
    }
    const auto health = ToProto(snapshot);
    const std::string bytes = health.SerializeAsString();
    const std::string tmp = path_ + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            return false;
        }
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        if (!out) {
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, path_, ec);
    if (ec) {
        AWARN << "HealthSnapshotStore: rename failed: " << ec.message();
        return false;
    }
    return true;
}

std::optional<::automsgs::rpcs::system::SystemHealth> HealthSnapshotStore::Read()
    const {
    std::ifstream in(path_, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::string bytes((std::istreambuf_iterator<char>(in)),
                      std::istreambuf_iterator<char>());
    ::automsgs::rpcs::system::SystemHealth health;
    if (!health.ParseFromString(bytes)) {
        AWARN << "HealthSnapshotStore: parse failed for " << path_;
        return std::nullopt;
    }
    return health;
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
