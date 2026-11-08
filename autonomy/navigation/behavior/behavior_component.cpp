/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/behavior_component.hpp"

#include <chrono>

namespace navigation
{
namespace behavior
{

BehaviorComponent::BehaviorComponent() = default;

BehaviorComponent::~BehaviorComponent()
{
    Clear();
}

bool BehaviorComponent::Init()
{
    status_reader_ =
        node_->CreateReader<NavigationStatus>("/navigation/status");
    heartbeat_writer_ =
        node_->CreateWriter<lifecycle::Heartbeat>("/behavior/heartbeat");
    return status_reader_ != nullptr && heartbeat_writer_ != nullptr;
}

bool BehaviorComponent::Proc()
{
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    lifecycle::Heartbeat heartbeat;
    heartbeat.set_module("behavior");
    heartbeat.set_sequence(++sequence_);
    heartbeat.set_stamp_ns(now_ns);
    heartbeat.set_state(lifecycle::MODULE_STATE_READY);
    if (const auto status = status_reader_->GetLatestObserved(); status != nullptr)
    {
        heartbeat.set_state(status->degraded() ? lifecycle::MODULE_STATE_DEGRADED
                                               : lifecycle::MODULE_STATE_READY);
    }
    return heartbeat_writer_->Write(heartbeat);
}

void BehaviorComponent::Clear()
{
    status_reader_.reset();
    heartbeat_writer_.reset();
}

}  // namespace behavior
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::behavior::BehaviorComponent)
