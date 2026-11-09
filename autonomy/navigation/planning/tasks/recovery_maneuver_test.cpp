/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/planning/tasks/recovery_maneuver.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

int main() {
    using namespace navigation;
    GoalSpec goal;
    goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(3.0);
    config::BehaviorOptions options;
    for (unsigned attempt : {1U, 2U}) {
        planning::RecoveryManeuver maneuver;
        common::Pose2D pose = common::MakePose2D(0.5, 0.0, 0.0);
        std::int64_t now = 1000000000LL;
        if (!maneuver.Start(pose, goal, options, attempt, now))
            return 1;
        // Independent differential-drive plant; verify the measured trajectory
        // goes around the obstacle, never through it, in either direction.
        double max_side = 0.0;
        for (int tick = 0; tick < 4000 && maneuver.State() == RECOVERY_RUNNING;
             ++tick) {
            now += 20000000LL;
            const auto command = maneuver.Tick(pose, now);
            if (command.x() < 0.0 || std::abs(command.y()) > 1e-9)
                return 2;
            pose.set_x(pose.x() + command.x() * std::cos(pose.theta()) * 0.02);
            pose.set_y(pose.y() + command.x() * std::sin(pose.theta()) * 0.02);
            pose.set_theta(pose.theta() + command.theta() * 0.02);
            const double closest_x = std::clamp(pose.x(), 1.0, 1.3);
            const double closest_y = std::clamp(pose.y(), -1.0, 1.0);
            if (std::hypot(pose.x() - closest_x, pose.y() - closest_y) < 0.3)
                return 3;
            max_side = std::max(max_side, std::abs(pose.y()));
        }
        if (maneuver.State() != RECOVERY_SUCCEEDED || pose.x() < 2.3 ||
            max_side < 1.8 || pose.y() * (attempt == 1 ? 1 : -1) < 1.8)
            return 4;
        maneuver.Reset();
        if (maneuver.State() != RECOVERY_IDLE ||
            std::abs(maneuver.Tick(pose, now).x()) > 1e-9)
            return 5;
    }
    // A stationary plant never completes a motion phase just because time
    // passes.
    planning::RecoveryManeuver blocked;
    auto pose = common::MakePose2D(0.5, 0.0, 0.0);
    blocked.Start(pose, goal, options, 1, 1000000000LL);
    blocked.Tick(pose, 1020000000LL);
    const auto zero = blocked.Tick(pose, 9100000000LL);
    if (blocked.State() != RECOVERY_FAILED ||
        std::abs(zero.x()) + std::abs(zero.theta()) > 1e-9)
        return 6;
    std::cout << "PASS odometry detour, opposite flanks, footprint clearance, "
                 "blocked timeout and reset\n";
    return 0;
}
