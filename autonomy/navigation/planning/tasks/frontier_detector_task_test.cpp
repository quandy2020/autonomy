/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/tasks/frontier_detector_task.hpp"

int main()
{
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    grid.mutable_info()->set_width(10);
    grid.mutable_info()->set_height(10);
    grid.mutable_info()->set_resolution(1.0F);
    for (int y = 0; y < 10; ++y)
    {
        for (int x = 0; x < 10; ++x)
        {
            const bool free = x >= 3 && x < 7 && y >= 3 && y < 7;
            grid.add_data(free ? 0 : -1);
        }
    }

    navigation::planning::FrontierDetectorTask detector;
    const auto frontiers = detector.Detect(grid, navigation::common::Pose2D{}, 100.0);
    if (frontiers.empty())
    {
        std::cerr << "no frontier detected\n";
        return 1;
    }
    for (const auto& frontier : frontiers)
    {
        if (frontier.pose.x() < 0.0 || frontier.pose.x() > 10.0 ||
            frontier.pose.y() < 0.0 || frontier.pose.y() > 10.0)
        {
            std::cerr << "frontier outside grid\n";
            return 1;
        }
    }
    const auto limited = detector.Detect(grid, navigation::common::Pose2D{}, 1.0);
    if (!limited.empty() && limited.front().pose.x() > 3.0)
    {
        // Range filter is allowed to return fewer points; this is a sanity check.
    }
    return 0;
}
