/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/plugins/reflex/reactive_avoider.hpp"
#include "navigation/planning/plugins/reflex/reflex_velocity_sampler.hpp"

int main()
{
    using namespace navigation;  // NOLINT

    plugins::ReactiveAvoiderOptions options;
    options.set_near_field_range(2.0);
    options.set_reaction_time_ms(30.0);
    options.set_safety_margin(0.3);
    options.set_fov_rad(1.57);
    options.set_candidate_count(16);
    planning::ReactiveAvoider avoider(options);
    const common::Twist2D desired = navigation::common::MakeTwist2D(0.4, 0.0, 0.0);

    ReflexObservation clear;
    clear.set_valid(true);
    const auto passthrough = avoider.Compute(clear, desired, 0.1);
    if (passthrough.intervened)
    {
        std::cerr << "clear path intervened\n";
        return 1;
    }

    ReflexObservation blocked;
    blocked.set_valid(true);
    auto* obstacle = blocked.add_obstacles();
    obstacle->set_x(0.2);
    obstacle->set_radius(0.1);
    const auto stop = avoider.Compute(blocked, desired, 0.1);
    if (!stop.intervened || stop.velocity.x() != 0.0)
    {
        std::cerr << "close obstacle not stopped\n";
        return 1;
    }

    planning::ReflexVelocitySampler sampler;
    const auto sampled = sampler.Sample(blocked, desired, 0.4);
    if (!sampled.has_value() || std::abs(sampled->theta()) <= 0.0)
    {
        std::cerr << "sampler failed\n";
        return 1;
    }
    return 0;
}
