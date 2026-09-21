/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file context.cpp
 * @brief Implementation of Context construction and CancelAllTasks / Estop.
 */

#include "autonomy/bridge/grpc/context.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

Context::Context(
    std::shared_ptr<autolink::Node> node,
    WorkScheduler::SharedPtr scheduler,
    proto::RobotIdentityOptions identity,
    proto::CapabilitiesOptions capabilities)
    : muxer_(TaskMuxer::make_shared()),
      idempotency_(CommandIdempotencyCache::make_unique()),
      work_scheduler_(std::move(scheduler)),
      state_hub_(StateHub::make_unique(node, muxer_)),
      domains_(node, muxer_, work_scheduler_.get(), idempotency_.get()),
      identity_(std::move(identity)),
      capabilities_(std::move(capabilities)) {
    if (!work_scheduler_) {
        AERROR << "Context: work scheduler is null";
    }
}

void Context::CancelAllTasks() {
    domains_.CancelAll();
    muxer_->Clear();
}

void Context::EmergencyStop(const bool engage) {
    muxer_->SetEstop(engage);
    ::autonomy::system::safety::SafetyLatch latch;
    latch.SetLatched(engage, engage ? "bridge_emergency_stop" : "");
    if (engage) {
        CancelAllTasks();
        muxer_->SetEstop(true);
    }
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
