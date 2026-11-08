/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/tasks/path_follower.hpp"

int main()
{
    navigation::planning::PathFollower follower;
    navigation::planning::Waypoint waypoint;
    waypoint.pose.set_x(1.0);

    navigation::common::Twist2D command;
    if (!follower.Compute(navigation::common::Pose2D{}, waypoint, &command))
    {
        std::cerr << "compute failed\n";
        return 1;
    }
    if (command.x() <= 0.0 || std::abs(command.theta()) > 1e-9)
    {
        std::cerr << "straight command failed\n";
        return 1;
    }
    waypoint.pose.set_y(1.0);
    if (!follower.Compute(navigation::common::Pose2D{}, waypoint, &command) ||
        command.theta() <= 0.0)
    {
        std::cerr << "turn command failed\n";
        return 1;
    }
    waypoint.pose = navigation::common::Pose2D{};
    waypoint.tolerance = 0.15;
    if (!follower.Reached(navigation::common::Pose2D{}, waypoint))
    {
        std::cerr << "reached failed\n";
        return 1;
    }
    return 0;
}
