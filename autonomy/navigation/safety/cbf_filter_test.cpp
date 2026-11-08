/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/platform_registry.hpp"
#include "navigation/safety/cbf_filter.hpp"

namespace
{

navigation::common::PlatformProfile Profile()
{
    navigation::common::PlatformProfile profile;
    profile.kinematic_type = navigation::common::KinematicType::kDifferential;
    profile.max_linear_mps = 2.0;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 1.0;
    profile.max_jerk_mps3 = 5.0;
    profile.footprint_radius = 0.3;
    return profile;
}

navigation::ReflexObservation Obstacle(double x)
{
    navigation::ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(1);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(x);
    obstacle->set_radius(0.2);
    return observation;
}

}  // namespace

int main()
{
    using navigation::common::Twist2D;
    using navigation::safety::CbfFilter;

    CbfFilter filter;
    if (!filter.Configure(Profile(), 0.1, 0.1))
    {
        std::cerr << "configure failed\n";
        return 1;
    }
    const auto observation = Obstacle(1.0);
    if (filter.IsSafe(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), observation))
    {
        std::cerr << "unsafe velocity accepted\n";
        return 1;
    }
    const auto projected = filter.Project(navigation::common::MakeTwist2D(1.0, 0.0, 0.0), observation);
    if (!projected.has_value() || projected->x() > 0.05)
    {
        std::cerr << "projection failed\n";
        return 1;
    }
    if (!filter.IsSafe(*projected, observation))
    {
        std::cerr << "projection unsafe\n";
        return 1;
    }
    const auto clear = Obstacle(-3.0);
    if (!filter.IsSafe(navigation::common::MakeTwist2D(1.0, 0.0, 0.0), clear))
    {
        std::cerr << "free space rejected\n";
        return 1;
    }
    return 0;
}
