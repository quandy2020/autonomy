/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/behavior_component.hpp"

#include "autolink/component/component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include "autolink/common/log.hpp"
#include "navigation/common/configuration_loader.hpp"

namespace navigation {
namespace behavior {

BehaviorComponent::BehaviorComponent() = default;

BehaviorComponent::~BehaviorComponent() {
    Clear();
}

bool BehaviorComponent::Init() {
    config::NavigationConfig configuration;
    if (!common::LoadTextProto(ConfigFilePath(), &configuration).Ok())
        return false;
    options_ = configuration.behavior();
    request_writer_ =
        node_->CreateWriter<BehaviorRequest>("/navigation/behavior_request");
    session_ = std::make_unique<BehaviorSession>(
        [this](const BehaviorRequest& request) {
            return request_writer_ && request_writer_->Write(request);
        });
    if (!engine_.Configure(options_, session_.get())) {
        AERROR << "Cannot load behavior XML: " << engine_.Error();
        return false;
    }
    engine_.Context().request_assist = [this](std::string_view reason) {
        (void)session_->FailGoal(reason);
    };
    status_reader_ = node_->CreateReader<NavigationStatus>(
        "/navigation/status", [this](const auto&) {
            status_received_ns_.store(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        });
    heartbeat_writer_ =
        node_->CreateWriter<lifecycle::Heartbeat>("/behavior/heartbeat");
    return status_reader_ != nullptr && heartbeat_writer_ != nullptr &&
           request_writer_ != nullptr;
}

bool BehaviorComponent::Proc() {
    status_reader_->Observe();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    lifecycle::Heartbeat heartbeat;
    heartbeat.set_module("behavior");
    heartbeat.set_sequence(++sequence_);
    heartbeat.set_stamp_ns(now_ns);
    heartbeat.set_state(lifecycle::MODULE_STATE_READY);
    if (const auto status = status_reader_->GetLatestObserved();
        status != nullptr) {
        const auto received = status_received_ns_.load();
        const bool fresh = received > 0 && now_ns >= received &&
                           now_ns - received <= 500000000LL &&
                           now_ns >= status->steady_stamp_ns() &&
                           now_ns - status->steady_stamp_ns() <= 500000000LL;
        if (session_id_ != status->session_id() ||
            goal_epoch_ != status->goal_epoch() || !status->has_active_goal()) {
            engine_.Reset();
            session_id_ = status->session_id();
            goal_epoch_ = status->goal_epoch();
            last_tick_ns_ = 0;
        }
        session_->Update(*status, now_ns);
        auto& context = engine_.Context();
        context.goal = status->goal();
        context.pose_valid = status->pose_valid();
        context.module_healthy = fresh;
        const double rate = std::isfinite(options_.tick_rate_hz()) &&
                                    options_.tick_rate_hz() > 0.0
                                ? options_.tick_rate_hz()
                                : 10.0;
        if (options_.enabled() && fresh && status->pose_valid() &&
            status->has_active_goal() && now_ns - last_tick_ns_ >= 1e9 / rate) {
            last_tick_ns_ = now_ns;
            if (context.assist_requested) {
                // Keep sending the terminal request until navigation
                // acknowledges it by clearing the goal. A single lost IPC
                // sample cannot hang it.
                (void)session_->FailGoal("recovery_exhausted");
            } else if (!engine_.Tick(now_ns).Ok() &&
                       !context.assist_requested) {
                (void)session_->FailGoal("behavior_failure");
            }
        }
        heartbeat.set_state(!fresh || context.assist_requested
                                ? lifecycle::MODULE_STATE_DEGRADED
                                : lifecycle::MODULE_STATE_READY);
    }
    return heartbeat_writer_->Write(heartbeat);
}

void BehaviorComponent::Clear() {
    autolink::TimerComponent::Clear();
    // Node also owns auxiliary readers; reset alone leaves their tasks alive.
    if (status_reader_ != nullptr)
        status_reader_->Shutdown();
    status_reader_.reset();
    engine_.Reset();
    session_.reset();
    request_writer_.reset();
    heartbeat_writer_.reset();
}

}  // namespace behavior
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::behavior::BehaviorComponent)
