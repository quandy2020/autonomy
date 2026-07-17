/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/system/monitor/monitor_base.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace system {
namespace monitor {

class HazardMonitor;

/// MRM 急停：Hazard ERROR 时向 cmd_vel 发布零速度（需显式启用）。
class MrmHandler : public MonitorBase
{
public:
    struct Options {
        std::string cmd_vel_channel{"/cmd_vel"};
        bool emergency_stop_on_error{true};
    };

    explicit MrmHandler(Options options);

    std::string Name() const override {
        return "mrm";
    }

    void SetHazardSource(HazardMonitor* hazard);
    bool AttachNode(const std::shared_ptr<autolink::Node>& node);
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    bool active() const {
        return active_;
    }

    static std::unique_ptr<MrmHandler> Create(Options options);

private:
    void PublishStop();

    Options options_;
    HazardMonitor* hazard_{nullptr};
    std::shared_ptr<autolink::Node> node_;
    bool attached_{false};
    bool active_{false};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    void* active_gauge_{nullptr};
#endif
};

std::unique_ptr<MrmHandler> CreateMrmHandler(MrmHandler::Options opts);

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
