/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/safety/velocity_feasibility_filter.hpp"

int main()
{
    using navigation::common::PlatformProfile;
    using navigation::common::Twist2D;
    using navigation::safety::VelocityFeasibilityFilter;

    PlatformProfile profile;
    profile.max_linear_mps = 0.5;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 0.5;
    profile.max_jerk_mps3 = 5.0;
    profile.footprint_radius = 0.3;

    VelocityFeasibilityFilter filter;
    if (!filter.Configure(profile))
    {
        std::cerr << "configure failed\n";
        return 1;
    }
    Twist2D filtered;
    if (!filter.Filter(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), navigation::common::MakeTwist2D(0.0, 0.0, 0.0), 0.1, &filtered))
    {
        std::cerr << "filter failed\n";
        return 1;
    }
    if (std::abs(filtered.x() - 0.05) > 1e-9)
    {
        std::cerr << "acceleration limit failed: " << filtered.x() << "\n";
        return 1;
    }
    if (filter.Filter(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), navigation::common::MakeTwist2D(0.0, 0.0, 0.0), 0.0, &filtered))
    {
        std::cerr << "zero dt accepted\n";
        return 1;
    }
    return 0;
}
