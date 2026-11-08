/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/scenarios/coverage/coverage_stage.hpp"

int main()
{
    navigation::Region region;
    const double vertices[4][2] = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    for (const auto& vertex : vertices)
    {
        auto* point = region.add_polygon();
        point->set_x(vertex[0]);
        point->set_y(vertex[1]);
    }

    navigation::planning::scenarios::CoverageStage stage;
    if (!stage.SetRegion(region) || stage.RemainingWaypoints() == 0)
    {
        std::cerr << "set region failed\n";
        return 1;
    }
    navigation::common::Twist2D command;
    if (!stage.Plan(navigation::common::Pose2D{}, &command) ||
        !std::isfinite(command.x()))
    {
        std::cerr << "plan failed\n";
        return 1;
    }
    stage.Reset();
    if (stage.RemainingWaypoints() != 0)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
