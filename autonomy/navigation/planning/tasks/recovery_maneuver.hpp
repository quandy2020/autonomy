/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/navigation.pb.h"

namespace navigation::planning {

/**
 * Odometry-controlled detour: face a flank, travel along it, face the goal
 * bearing, advance, then hand back to the original policy. Commands are raw
 * intents: the caller must pass every command through Safety. No blind reverse
 * or lateral translation is used. Odd/even attempts alternate flanks.
 */
class RecoveryManeuver
{
public:
    bool Start(const common::Pose2D& pose, const GoalSpec& goal,
               const config::BehaviorOptions& options, std::uint32_t attempt,
               std::int64_t now_ns);
    common::Twist2D Tick(const common::Pose2D& pose, std::int64_t now_ns);
    void Reset();
    RecoveryState State() const {
        return state_;
    }
    std::string_view Phase() const;

private:
    std::array<common::Pose2D, 4> targets_;
    std::size_t phase_{0};
    RecoveryState state_{RECOVERY_IDLE};
    std::int64_t phase_started_ns_{0}, last_progress_ns_{0}, last_tick_ns_{0};
    std::int64_t phase_timeout_ns_{25000000000LL},
        progress_timeout_ns_{8000000000LL};
    double best_error_{1e9};
};

}  // namespace navigation::planning
