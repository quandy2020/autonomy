/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/system_monitor_stub.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/system/monitor/monitor_options.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

using ::autonomy::system::monitor::HazardLevel;
using ::autonomy::system::monitor::LoadMonitorOptions;
using ::autonomy::system::monitor::MonitorOptions;
namespace system_rpc = ::automsgs::rpcs::system;

system_rpc::HazardLevel ConvertHazardLevel(HazardLevel level) {
    switch (level) {
        case HazardLevel::kWarn:
            return system_rpc::HAZARD_LEVEL_WARN;
        case HazardLevel::kError:
            return system_rpc::HAZARD_LEVEL_ERROR;
        case HazardLevel::kOk:
        default:
            return system_rpc::HAZARD_LEVEL_OK;
    }
}

MonitorOptions BuildBridgeMonitorOptions() {
    MonitorOptions options = LoadMonitorOptions("config", "system/monitor.lua");
    options.enable_prometheus = false;
    options.enable_mrm_handler = false;
    options.enable_cpu_profile = false;
    options.enable_heap_profile = false;
    return options;
}

}  // namespace

system_rpc::SystemHealth SystemMonitorTraits::ConvertToHealth(
    const ::autonomy::system::monitor::SystemHealthSnapshot& snapshot,
    const TaskMuxer* muxer, bool include_channel_lists) {
    system_rpc::SystemHealth health;
    health.set_hazard_level(ConvertHazardLevel(snapshot.hazard_level));
    health.set_mrm_active(snapshot.mrm_active);
    health.set_emergency_stop_latched(muxer && muxer->CheckEstopActive());
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

    if (include_channel_lists) {
        for (const auto& channel : snapshot.channels) {
            auto* channel_info = health.add_channels();
            channel_info->set_channel(channel.channel);
            channel_info->set_ever_received(channel.ever_received);
            channel_info->set_healthy(channel.healthy);
            channel_info->set_age_seconds(static_cast<float>(channel.age_sec));
            channel_info->set_rate_hz(static_cast<float>(channel.rate_hz));
        }
        for (const auto& latency : snapshot.latencies) {
            auto* latency_info = health.add_latencies();
            latency_info->set_channel(latency.channel);
            latency_info->set_ever_received(latency.ever_received);
            latency_info->set_healthy(latency.healthy);
            latency_info->set_message_age_seconds(
                static_cast<float>(latency.message_age_sec));
        }
    }
    return health;
}

SystemMonitorStub::SystemMonitorStub(std::shared_ptr<autolink::Node> node,
                                     std::shared_ptr<TaskMuxer> muxer)
    : node_(std::move(node)), muxer_(std::move(muxer)) {}

SystemMonitorStub::~SystemMonitorStub() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (registry_) {
        registry_->Stop();
        registry_.reset();
    }
}

void SystemMonitorStub::EnsureMonitorStarted() {
    if (started_) {
        return;
    }
    if (!node_) {
        AWARN << "SystemMonitorStub: no autolink node";
        return;
    }
    registry_ = std::make_unique<::autonomy::system::monitor::MonitorRegistry>(
        BuildBridgeMonitorOptions());
    registry_->AttachAutolinkNode(node_);
    registry_->Start();
    started_ = true;
    AINFO << "SystemMonitorStub: embedded MonitorRegistry started";
}

system_rpc::SystemHealth SystemMonitorStub::GetHealth(bool include_channel_lists) {
    std::lock_guard<std::mutex> lock(mutex_);
    EnsureMonitorStarted();
    if (!registry_) {
        system_rpc::SystemHealth health;
        health.set_hazard_level(system_rpc::HAZARD_LEVEL_UNKNOWN);
        health.set_emergency_stop_latched(muxer_ &&
                                          muxer_->CheckEstopActive());
        health.set_detail("monitor unavailable");
        return health;
    }
    registry_->CollectAll();
    return SystemMonitorTraits::ConvertToHealth(
        registry_->Snapshot(), muxer_.get(), include_channel_lists);
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
