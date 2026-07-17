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

class ChannelMonitor : public MonitorBase
{
public:
    explicit ChannelMonitor(std::vector<ChannelWatchOptions> watches);
    ~ChannelMonitor() override;

    std::string Name() const override {
        return "channel";
    }

    bool AttachNode(const std::shared_ptr<autolink::Node>& node);
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    std::vector<ChannelHealth> SnapshotHealth() const;

    static std::unique_ptr<ChannelMonitor> Create(
        std::vector<ChannelWatchOptions> watches);

private:
    struct ChannelRuntime;

    std::vector<ChannelWatchOptions> watches_;
    std::vector<std::unique_ptr<ChannelRuntime>> runtimes_;
    std::shared_ptr<autolink::Node> node_;
    bool attached_{false};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    struct PrometheusGauges;
    std::unique_ptr<PrometheusGauges> gauges_;
#endif
};

std::unique_ptr<ChannelMonitor> CreateChannelMonitor(
    std::vector<ChannelWatchOptions> watches);

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
