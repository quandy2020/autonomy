/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file safety_filter.hpp
 * @brief Post-policy Twist clamping, rate limiting, and collision check.
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class CommandSafetyFilter
 * @brief Last-mile safety layer applied to decoded policy actions.
 *
 * Order: speed scale → velocity/accel limits → footprint or point collision
 * along the commanded motion. Thread-safe Configure vs Apply via mutex.
 */
class CommandSafetyFilter
{
public:
    void Configure(const proto::PolicyControllerOptions& options,
                   const proto::TwistLimits& limits);

    void SetSpeedScale(double scale);

    uint32_t Apply(const automsgs::msgs::geometry_msgs::Twist& action,
                   const ObservationSnapshot& snapshot,
                   map::costmap_2d::Costmap2DWrapper* costmap_wrapper,
                   double elapsed_seconds,
                   automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
                   std::string* message, bool* clamp_or_rate_limit = nullptr,
                   bool* collision_blocked = nullptr);

    uint32_t ApplyFailure(
        const std::string& reason,
        automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
        std::string* message, bool force_zero = false);

    void Reset();

private:
    bool MotionCollisionBlocked(
        const ObservationSnapshot& snapshot,
        map::costmap_2d::Costmap2DWrapper* costmap_wrapper, double linear_x,
        double linear_y, double angular_z) const;

    mutable std::mutex mutex_;
    proto::TwistLimits limits_;
    bool hold_last_command_on_failure_{true};
    bool enable_collision_check_{false};
    bool use_footprint_collision_{true};
    double collision_check_distance_{0.5};
    double collision_cost_threshold_{253.0};
    double speed_scale_{1.0};
    std::optional<automsgs::msgs::geometry_msgs::Twist> last_command_;
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
