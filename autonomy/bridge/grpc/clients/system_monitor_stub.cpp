/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/system_monitor_stub.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace system_rpc = ::automsgs::rpcs::system;

}  // namespace

SystemMonitorStub::SystemMonitorStub(std::shared_ptr<autolink::Node> node,
                                     TaskMuxer::SharedPtr muxer)
    : node_(std::move(node)), muxer_(std::move(muxer)) {}

::automsgs::rpcs::system::SystemHealth SystemMonitorStub::GetHealth(
    bool include_channel_lists) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto health_opt = store_.Read();
    system_rpc::SystemHealth health;
    if (!health_opt) {
        health.set_hazard_level(system_rpc::HAZARD_LEVEL_UNKNOWN);
        health.set_detail(
            "health snapshot unavailable — is autonomy.monitor running?");
        (void)node_;
    } else {
        health = std::move(*health_opt);
        if (!include_channel_lists) {
            health.clear_channels();
            health.clear_latencies();
        }
    }

    const bool muxer_estop = muxer_ && muxer_->IsEstop();
    const bool file_estop =
        ::autonomy::system::safety::SafetyLatch{}.IsLatched();
    health.set_emergency_stop_latched(muxer_estop || file_estop ||
                                      health.emergency_stop_latched());
    return health;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
