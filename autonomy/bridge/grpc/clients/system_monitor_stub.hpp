/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file system_monitor_stub.hpp
 * @brief SystemMonitorStub: SystemService health via published snapshot (read-only).
 */

#pragma once

#include <memory>
#include <mutex>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/system/monitor/health_snapshot_store.hpp"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief SystemService GetHealth facade — reads HealthSnapshotStore only.
 * Does NOT embed a second MonitorRegistry (single health truth).
 */
class SystemMonitorStub
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SystemMonitorStub)

    SystemMonitorStub(std::shared_ptr<autolink::Node> node,
                      TaskMuxer::SharedPtr muxer);

    ~SystemMonitorStub() = default;

    ::automsgs::rpcs::system::SystemHealth GetHealth(
        bool include_channel_lists = true);

private:
    std::shared_ptr<autolink::Node> node_{nullptr};
    TaskMuxer::SharedPtr muxer_{nullptr};
    ::autonomy::system::monitor::HealthSnapshotStore store_;
    mutable std::mutex mutex_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
