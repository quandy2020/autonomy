/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file behavior_component.hpp
 * @brief autolink timer component publishing behavior health.
 */

#pragma once

#include <atomic>
#include <memory>
#include "navigation/behavior/engine/behavior_session.hpp"
#include "navigation/behavior/engine/bt_cpp_engine.hpp"

#include "autolink/component/timer_component.hpp"
#include "autonomy/common/macros.hpp"
#include "navigation/proto/lifecycle.pb.h"
#include "navigation/proto/navigation.pb.h"

namespace navigation {
namespace behavior {

/** @brief XML behavior runtime supervising the navigation session over IPC. */
class BehaviorComponent final : public autolink::TimerComponent
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(BehaviorComponent)

    /** @brief Construct an uninitialized component. */
    BehaviorComponent();

    /** @brief Release endpoints. */
    ~BehaviorComponent() override;

    /** @brief Release endpoints. */
    void Clear() override;

protected:
    /** @brief Create readers/writer. */
    bool Init() override;

private:
    /** @brief Publish one heartbeat from the latest navigation status. */
    bool Proc() override;

    /** @brief /navigation/status reader. */
    std::shared_ptr<autolink::Reader<NavigationStatus>> status_reader_{nullptr};
    /** @brief /behavior/heartbeat writer. */
    std::shared_ptr<autolink::Writer<lifecycle::Heartbeat>> heartbeat_writer_{
        nullptr};
    /** @brief Monotonic heartbeat sequence. */
    std::uint64_t sequence_{0};
    std::shared_ptr<autolink::Writer<BehaviorRequest>> request_writer_;
    std::unique_ptr<BehaviorSession> session_;
    BtCppEngine engine_;
    config::BehaviorOptions options_;
    std::atomic<std::int64_t> status_received_ns_{0};
    std::string session_id_;
    std::uint64_t goal_epoch_{0};
    std::int64_t last_tick_ns_{0};
};

}  // namespace behavior
}  // namespace navigation
