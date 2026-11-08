/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/tasks/coverage_planner.hpp"

int main()
{
    navigation::Region region;
    const double vertices[4][2] = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    for (const auto& point : vertices)
    {
        auto* vertex = region.add_polygon();
        vertex->set_x(point[0]);
        vertex->set_y(point[1]);
    }
    navigation::planning::CoveragePlanner planner(
        navigation::planning::CoverageOptions{0.5, 0.15});
    std::vector<navigation::planning::Waypoint> waypoints;
    if (!planner.Plan(region, &waypoints) || waypoints.empty())
    {
        std::cerr << "coverage plan failed\n";
        return 1;
    }
    for (const auto& waypoint : waypoints)
    {
        if (waypoint.pose.x() < 0.0 || waypoint.pose.x() > 2.0 ||
            waypoint.pose.y() < 0.0 || waypoint.pose.y() > 2.0)
        {
            std::cerr << "waypoint outside region\n";
            return 1;
        }
    }
    navigation::Region empty;
    if (planner.Plan(empty, &waypoints))
    {
        std::cerr << "empty region accepted\n";
        return 1;
    }
    return 0;
}
