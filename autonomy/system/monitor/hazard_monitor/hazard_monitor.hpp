/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>

#include "autonomy/system/monitor/monitor_base.hpp"
#include "autonomy/system/monitor/ops_types.hpp"

namespace autonomy {
namespace system {
namespace monitor {

class ChannelMonitor;
class LatencyMonitor;

/// 聚合 channel / latency 健康为统一 Hazard 等级（无 ROS diagnostic_graph）。
class HazardMonitor : public MonitorBase
{
public:
    HazardMonitor() = default;

    std::string Name() const override {
        return "hazard";
    }

    void SetSources(ChannelMonitor* channel, LatencyMonitor* latency);
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    HazardLevel level() const {
        return level_;
    }

    static std::unique_ptr<HazardMonitor> Create();

private:
    ChannelMonitor* channel_{nullptr};
    LatencyMonitor* latency_{nullptr};
    HazardLevel level_{HazardLevel::kOk};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    void* level_gauge_{nullptr};
#endif
};

std::unique_ptr<HazardMonitor> CreateHazardMonitor();

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
