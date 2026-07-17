/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "autonomy/system/monitor/monitor_base.hpp"
#include "autonomy/system/monitor/ops_types.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace system {
namespace monitor {

/// 基于消息 header.stamp 与当前时间的管线延迟（适用于 TwistStamped channel）。
class LatencyMonitor : public MonitorBase
{
public:
    explicit LatencyMonitor(std::vector<LatencyWatchOptions> watches);
    ~LatencyMonitor() override;

    std::string Name() const override {
        return "latency";
    }

    bool AttachNode(const std::shared_ptr<autolink::Node>& node);
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    std::vector<LatencyHealth> SnapshotHealth() const;

    static std::unique_ptr<LatencyMonitor> Create(
        std::vector<LatencyWatchOptions> watches);

private:
    struct LatencyRuntime;

    std::vector<LatencyWatchOptions> watches_;
    std::vector<std::unique_ptr<LatencyRuntime>> runtimes_;
    std::shared_ptr<autolink::Node> node_;
    bool attached_{false};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    struct PrometheusGauges;
    std::unique_ptr<PrometheusGauges> gauges_;
#endif
};

std::unique_ptr<LatencyMonitor> CreateLatencyMonitor(
    std::vector<LatencyWatchOptions> watches);

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
