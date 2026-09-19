/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file system_monitor_stub.hpp
 * @brief SystemMonitorStub: SystemService health via embedded MonitorRegistry.
 *
 * @details
 * Not a GoalChannel client. Lazily constructs
 * `autonomy::system::monitor::MonitorRegistry` on the first GetHealth call,
 * attaches the Autolink node, and converts SystemHealthSnapshot into
 * `automsgs.rpcs.system.SystemHealth`. Estop latch is overlaid from TaskMuxer
 * (`muxer->IsEstop()`), not from the monitor snapshot alone.
 *
 * Wire / config:
 * - Loads monitor options from `monitor.pb.txt` via LoadMonitorOptions
 * - Bridge disables prometheus / MRM / CPU / heap profiling in-process
 * - No dedicated bridge/constants.hpp topic; channel lists come from the
 * monitor snapshot when @c include_channel_lists is true
 *
 * @par Invariants
 * - StartMonitorIfNeeded is idempotent under mutex_.
 * - Destructor Stop()s and resets registry_ under mutex_.
 * - GetHealth never throws; returns HAZARD_LEVEL_UNKNOWN when registry missing.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; owns unique_ptr MonitorRegistry.
 *
 * @par Threading
 * GetHealth / destructor serialize on mutex_; MonitorRegistry may
 * spawn its own workers after Start().
 *
 * @see TaskMuxer
 * @see rpc_system_handlers.hpp
 */

#pragma once

#include <memory>
#include <mutex>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief SystemService GetHealth facade (lazy MonitorRegistry). Not GoalChannel.
 *
 * @details
 * Bridges the in-process system monitor into RPC SystemHealth for
 * SystemService/GetHealth. First call starts the registry; subsequent calls
 * sample Snapshot() and convert host / channel / latency fields.
 *
 * @par Threading
 * all public entry points take mutex_. Do not call GetHealth from
 * monitor callbacks that already hold registry locks (risk of lock inversion).
 *
 * @par Ownership
 * unique ownership of MonitorRegistry; UniquePtr of this stub held
 * by DomainBundle.
 *
 * @note Not registered as a CancelRegistry domain hook (health is read-only).
 * @warning Starting the registry requires a non-null Autolink node; otherwise
 * GetHealth returns a degraded "monitor unavailable" payload.
 *
 * @see rpc_system_handlers.hpp
 */
class SystemMonitorStub
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(SystemMonitorStub)

    /**
     * @brief Construct without starting the monitor (lazy start on GetHealth).
     *
     * @param[in] node  Autolink node attached to MonitorRegistry.
     * @param[in] muxer Shared muxer for emergency_stop_latched overlay.
     */
    SystemMonitorStub(std::shared_ptr<autolink::Node> node,
                      TaskMuxer::SharedPtr muxer);

    /**
     * @brief Stop the MonitorRegistry if started.
     *
     * @note Safe to destroy while gRPC handlers may still hold Context SharedPtr
     * only if GetHealth is no longer invoked after teardown begins.
     */
    ~SystemMonitorStub();

    /**
     * @brief Sample system health (lazy-starts MonitorRegistry).
     *
     * @param[in] include_channel_lists When true, populate channels / latencies
     *                                 repeated fields from the snapshot; when false, omit them for
     *                                 a lighter payload.
     * @return SystemHealth (hazard, host metrics, optional channel lists, estop).
     *
     * @note emergency_stop_latched always reflects muxer_->IsEstop() when muxer
     * is non-null.
     */
    ::automsgs::rpcs::system::SystemHealth GetHealth(
        bool include_channel_lists = true);

private:
    /**
     * @brief Construct, attach, and Start MonitorRegistry once.
     *
     * @warning Caller must hold mutex_. No-ops if already started or node_ null.
     */
    void StartMonitorIfNeeded();

    /**
     * @brief Autolink node attached to MonitorRegistry on first Start.
     *
     * @details Non-owning shared handle; must outlive GetHealth while the
     * registry is running. Null → GetHealth returns degraded payload.
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Shared TaskMuxer for emergency_stop_latched overlay.
     *
     * @details Non-owning SharedPtr into Context; may be null (estop field
     * omitted / false when absent).
     */
    TaskMuxer::SharedPtr muxer_{nullptr};

    /**
     * @brief Lazily constructed in-process MonitorRegistry (sole owner).
     *
     * @details Created and Start()'d under mutex_ on first GetHealth;
     * Stop()'d and reset in the destructor.
     */
    std::unique_ptr<::autonomy::system::monitor::MonitorRegistry> registry_{nullptr};

    /**
     * @brief Serializes GetHealth / StartMonitorIfNeeded / destructor.
     *
     * @warning Do not call GetHealth from monitor callbacks that already
     * hold registry locks (lock-inversion risk).
     */
    mutable std::mutex mutex_;

    /**
     * @brief True after MonitorRegistry has been successfully Start()'d.
     *
     * @details Makes StartMonitorIfNeeded idempotent under mutex_.
     */
    bool started_{false};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
