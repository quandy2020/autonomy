/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/rules/dynamic_obstacle_rule.hpp"
#include "navigation/planning/rules/forbidden_zone_rule.hpp"
#include "navigation/planning/rules/pedestrian_rule.hpp"
#include "navigation/planning/rules/speed_limit_rule.hpp"

int main()
{
    using namespace navigation;  // NOLINT

    planning::NavigationFrame frame;
    common::PlatformProfile profile;
    profile.max_linear_mps = 0.5;
    profile.max_angular_radps = 1.0;
    profile.max_lateral_mps = 0.0;
    profile.max_acceleration_mps2 = 0.8;
    profile.max_jerk_mps3 = 2.0;
    profile.footprint_radius = 0.3;
    frame.platform = &profile;

    common::Twist2D command = navigation::common::MakeTwist2D(1.0, 0.0, 2.0);
    planning::SpeedLimitRule speed;
    if (!speed.Apply(frame, &command).Ok() || command.x() > 0.5 + 1e-9 ||
        command.theta() > 1.0 + 1e-9)
    {
        std::cerr << "speed limit failed\n";
        return 1;
    }

    planning::ForbiddenZoneRule zone;
    navigation::Region region;
    for (const auto& point : {{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}})
    {
        auto* vertex = region.add_polygon();
        vertex->set_x(point[0]);
        vertex->set_y(point[1]);
    }
    zone.SetZones({region});
    command = navigation::common::MakeTwist2D(0.3, 0.0, 0.0);
    if (zone.Apply(frame, &command).Ok() || command.x() != 0.0)
    {
        std::cerr << "forbidden zone failed\n";
        return 1;
    }

    ReflexObservation observation;
    observation.set_valid(true);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(0.35);
    obstacle->set_radius(0.05);
    frame.pose = common::Pose2D{};
    frame.reflex = &observation;
    command = navigation::common::MakeTwist2D(0.4, 0.0, 0.0);
    planning::DynamicObstacleRule dynamic;
    if (dynamic.Apply(frame, &command).Ok() || command.x() != 0.0)
    {
        std::cerr << "dynamic rule failed\n";
        return 1;
    }
    command = navigation::common::MakeTwist2D(0.4, 0.0, 0.0);
    planning::PedestrianRule pedestrian;
    if (pedestrian.Apply(frame, &command).Ok())
    {
        std::cerr << "pedestrian rule failed\n";
        return 1;
    }
    return 0;
}
