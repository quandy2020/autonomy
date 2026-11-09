/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/planning/tasks/recovery_maneuver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation::planning {
namespace {
double Wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
double PositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}
common::Pose2D Pose(double x, double y, double yaw) {
    common::Pose2D pose;
    pose.set_x(x);
    pose.set_y(y);
    pose.set_theta(Wrap(yaw));
    return pose;
}
}  // namespace

bool RecoveryManeuver::Start(const common::Pose2D& pose, const GoalSpec& goal,
                             const config::BehaviorOptions& options,
                             std::uint32_t attempt, std::int64_t now_ns) {
    Reset();
    if (!common::IsFinite(pose) || !goal.has_pose() || attempt == 0)
        return false;
    const auto& target = goal.pose().pose().position();
    const double dx = target.x() - pose.x(), dy = target.y() - pose.y();
    if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) < 0.15)
        return false;
    const double heading = std::atan2(dy, dx);
    const double sign = attempt % 2 == 1 ? 1.0 : -1.0;
    const double lateral =
        PositiveOr(options.recovery_lateral_distance_m(), 2.0);
    const double forward =
        PositiveOr(options.recovery_forward_distance_m(), 2.0);
    const double side_yaw = heading + sign * 1.5707963267948966;
    const double side_x = pose.x() - sign * lateral * std::sin(heading);
    const double side_y = pose.y() + sign * lateral * std::cos(heading);
    targets_[0] = Pose(pose.x(), pose.y(), side_yaw);
    targets_[1] = Pose(side_x, side_y, side_yaw);
    targets_[2] = Pose(side_x, side_y, heading);
    targets_[3] = Pose(side_x + forward * std::cos(heading),
                       side_y + forward * std::sin(heading), heading);
    phase_timeout_ns_ = static_cast<std::int64_t>(
        1e9 * PositiveOr(options.recovery_phase_timeout_s(), 25.0));
    progress_timeout_ns_ = static_cast<std::int64_t>(
        1e9 * PositiveOr(options.no_progress_timeout_s(), 8.0));
    phase_started_ns_ = last_progress_ns_ = last_tick_ns_ = now_ns;
    state_ = RECOVERY_RUNNING;
    return true;
}

common::Twist2D RecoveryManeuver::Tick(const common::Pose2D& pose,
                                       std::int64_t now_ns) {
    common::Twist2D command;
    if (state_ != RECOVERY_RUNNING)
        return command;
    if (!common::IsFinite(pose) || now_ns < last_tick_ns_) {
        state_ = RECOVERY_FAILED;
        return command;
    }
    last_tick_ns_ = now_ns;
    const auto& target = targets_[phase_];
    const double distance =
        std::hypot(target.x() - pose.x(), target.y() - pose.y());
    const double yaw_error = Wrap(target.theta() - pose.theta());
    const bool rotate = phase_ % 2 == 0;
    const double error = rotate ? std::abs(yaw_error) : distance;
    if (error < (rotate ? 0.08 : 0.10)) {
        if (++phase_ == targets_.size()) {
            state_ = RECOVERY_SUCCEEDED;
        } else {
            phase_started_ns_ = last_progress_ns_ = now_ns;
            best_error_ = 1e9;
        }
        return command;
    }
    if (error < best_error_ - 0.04) {
        best_error_ = error;
        last_progress_ns_ = now_ns;
    }
    if (now_ns - phase_started_ns_ > phase_timeout_ns_ ||
        now_ns - last_progress_ns_ > progress_timeout_ns_) {
        state_ = RECOVERY_FAILED;
        return command;
    }
    const double heading_error =
        rotate ? yaw_error
               : Wrap(std::atan2(target.y() - pose.y(), target.x() - pose.x()) -
                      pose.theta());
    // Turn before translating; forward depth then covers the intended path.
    if (!rotate && std::abs(heading_error) < 0.20) {
        command.set_x(static_cast<float>(std::min(0.30, 0.6 * distance)));
    }
    command.set_theta(
        static_cast<float>(std::clamp(1.5 * heading_error, -0.6, 0.6)));
    return command;
}

void RecoveryManeuver::Reset() {
    state_ = RECOVERY_IDLE;
    phase_ = 0;
    best_error_ = 1e9;
    phase_started_ns_ = last_progress_ns_ = last_tick_ns_ = 0;
}

std::string_view RecoveryManeuver::Phase() const {
    if (state_ != RECOVERY_RUNNING)
        return "idle";
    constexpr std::array<std::string_view, 4> names{"face_flank", "flank",
                                                    "face_goal", "advance"};
    return names[phase_];
}
}  // namespace navigation::planning
