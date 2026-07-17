/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/hazard_monitor/hazard_monitor.hpp"

#include "autonomy/system/monitor/channel_monitor/channel_monitor.hpp"
#include "autonomy/system/monitor/latency_monitor/latency_monitor.hpp"

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

std::unique_ptr<HazardMonitor> HazardMonitor::Create() {
    return std::make_unique<HazardMonitor>();
}

std::unique_ptr<HazardMonitor> CreateHazardMonitor() {
    return HazardMonitor::Create();
}

void HazardMonitor::SetSources(ChannelMonitor* channel,
                               LatencyMonitor* latency) {
    channel_ = channel;
    latency_ = latency;
}

void HazardMonitor::Collect() {
    bool any_error = false;
    bool any_warn = false;

    if (channel_) {
        for (const auto& h : channel_->SnapshotHealth()) {
            if (!h.ever_received && h.channel.find("cmd_vel") != std::string::npos) {
                any_warn = true;
            } else if (h.ever_received && !h.healthy) {
                any_error = true;
            }
        }
    }
    if (latency_) {
        for (const auto& h : latency_->SnapshotHealth()) {
            if (h.ever_received && !h.healthy)
                any_error = true;
        }
    }

    if (any_error)
        level_ = HazardLevel::kError;
    else if (any_warn)
        level_ = HazardLevel::kWarn;
    else
        level_ = HazardLevel::kOk;

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (level_gauge_)
        static_cast<prometheus::Gauge*>(level_gauge_)
            ->Set(static_cast<double>(static_cast<int>(level_)));
#endif
}

void HazardMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    auto& family = prometheus::BuildGauge()
                       .Name("autonomy_system_hazard_level")
                       .Help("0=OK 1=WARN 2=ERROR")
                       .Register(*reg);
    level_gauge_ = &family.Add({});
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
