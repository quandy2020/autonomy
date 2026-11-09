/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/safety/aeb_velocity_sampler.hpp"

namespace
{

navigation::common::PlatformProfile Profile()
{
    navigation::common::PlatformProfile profile;
    profile.kinematic_type = navigation::common::KinematicType::kDifferential;
    profile.max_linear_mps = 0.5;
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
    obstacle->set_radius(0.1);
    return observation;
}

}  // namespace

int main()
{
    using navigation::common::Twist2D;
    using navigation::safety::AebVelocitySampler;
    using navigation::safety::CbfFilter;

    const auto profile = Profile();
    CbfFilter filter;
    filter.Configure(profile, 2.0, 0.05);

    AebVelocitySampler sampler;
    const auto far = sampler.Sample(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), Obstacle(5.0), profile, filter, 8);
    if (!far.has_value() || far->x() > 0.5)
    {
        std::cerr << "far sample failed\n";
        return 1;
    }
    if (filter.IsSafe(navigation::common::MakeTwist2D(0.0, 0.0, 0.0), Obstacle(0.1)))
    {
        std::cerr << "inside obstacle reported safe\n";
        return 1;
    }
    const auto blocked = sampler.Sample(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), Obstacle(0.1), profile,
                                        filter, 8);
    if (blocked.has_value())
    {
        std::cerr << "blocked sample should be empty\n";
        return 1;
    }
    if (filter.IsSafe(navigation::common::MakeTwist2D(0, 0, .5), Obstacle(0.1)) ||
        filter.IsSafe(navigation::common::MakeTwist2D(0, 0, 0), Obstacle(0))) {
        std::cerr << "deep overlap or origin obstacle reported safe\n";
        return 1;
    }
    // Contact recovery remains possible when retreat satisfies the barrier.
    if (!filter.IsSafe(navigation::common::MakeTwist2D(-.3, 0, 0), Obstacle(.35))) return 1;
    if (filter.Project(navigation::common::MakeTwist2D(0, 0, .5), Obstacle(.35)).has_value() ||
        !filter.Project(navigation::common::MakeTwist2D(-.3, 0, 0), Obstacle(.35)).has_value()) {
        std::cerr << "overlap projected blind reverse or rejected explicit retreat\n";
        return 1;
    }
    return 0;
}
