/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <mutex>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"
#include "autonomy/system/monitor/system_health_snapshot.hpp"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Conversion policy for embedded MonitorRegistry snapshots.
 */
struct SystemMonitorTraits {
    /**
     * @brief Convert a monitor snapshot into the RPC SystemHealth message.
     * @param[in] snapshot Internal snapshot.
     * @param[in] muxer Optional muxer for estop latch.
     * @param[in] include_channel_lists Whether to fill channel/latency arrays.
     */
    static ::automsgs::rpcs::system::SystemHealth ConvertToHealth(
        const ::autonomy::system::monitor::SystemHealthSnapshot& snapshot,
        const TaskMuxer* muxer, bool include_channel_lists);
};

/**
 * @brief Lazy-started embedded MonitorRegistry facade for SystemService.
 */
class SystemMonitorStub
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SystemMonitorStub)

    /**
     * @brief Construct without starting the registry (lazy on first GetHealth).
     * @param[in] node Autolink node attached to the registry.
     * @param[in] muxer Shared task muxer (estop latch).
     */
    SystemMonitorStub(std::shared_ptr<autolink::Node> node,
                      std::shared_ptr<TaskMuxer> muxer);

    ~SystemMonitorStub();

    /**
     * @brief Collect and return system health.
     * @param[in] include_channel_lists Include per-channel stats when true.
     */
    ::automsgs::rpcs::system::SystemHealth GetHealth(
        bool include_channel_lists = true);

private:
    /** @brief Start MonitorRegistry once under lock. */
    void EnsureMonitorStarted();

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<TaskMuxer> muxer_;
    std::unique_ptr<::autonomy::system::monitor::MonitorRegistry> registry_;
    mutable std::mutex mutex_;
    bool started_{false};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
