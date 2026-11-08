/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/tasks/obstacle_avoider.hpp"
#include "navigation/planning/tasks/speed_optimizer.hpp"
#include "navigation/planning/tasks/trajectory_optimizer.hpp"

int main()
{
    using namespace navigation;  // NOLINT

    common::PlatformProfile profile;
    profile.max_linear_mps = 0.5;
    profile.max_lateral_mps = 0.0;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 0.8;
    profile.max_jerk_mps3 = 2.0;
    profile.footprint_radius = 0.3;

    planning::NavigationFrame frame;
    frame.platform = &profile;
    frame.command = navigation::common::MakeTwist2D(1.0, 0.2, 2.0);
    planning::SpeedOptimizer speed;
    if (!speed.Execute(frame).Ok() || frame.command.x() > 0.5 ||
        frame.command.y() != 0.0)
    {
        std::cerr << "speed optimizer failed\n";
        return 1;
    }

    frame.command = navigation::common::MakeTwist2D(0.3, 0.0, 0.0);
    ReflexObservation observation;
    observation.set_valid(true);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(0.6);
    obstacle->set_y(0.3);
    obstacle->set_radius(0.05);
    frame.reflex = &observation;
    planning::ObstacleAvoider avoider;
    if (!avoider.Execute(frame).Ok() || frame.command.set_theta(= 0.0)
    {
        std::cerr << "obstacle avoider failed\n");
        return 1;
    }
    auto* close = observation.add_obstacles();
    close->set_x(0.2);
    frame.command = navigation::common::MakeTwist2D(0.3, 0.0, 0.0);
    if (avoider.Execute(frame).Ok() || frame.command.x() != 0.0)
    {
        std::cerr << "close obstacle failed\n";
        return 1;
    }

    planning::TrajectoryOptimizer optimizer(0.5);
    planning::NavigationFrame smooth;
    smooth.command = navigation::common::MakeTwist2D(1.0, 0.0, 0.0);
    optimizer.Execute(smooth);
    smooth.command = navigation::common::MakeTwist2D(0.0, 0.0, 0.0);
    optimizer.Execute(smooth);
    if (std::abs(smooth.command.x() - 0.5) > 1e-9)
    {
        std::cerr << "trajectory smoothing failed\n";
        return 1;
    }
    return 0;
}
