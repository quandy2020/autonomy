/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/safety/reflex_policy_arbiter.hpp"

namespace
{

navigation::ReflexObservation Obstacle(double x, double vx_mps = 0.0)
{
    navigation::ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(1);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(x);
    obstacle->set_vx_mps(vx_mps);
    obstacle->set_radius(0.05);
    return observation;
}

navigation::plugins::ReactiveAvoiderOptions Options()
{
    navigation::plugins::ReactiveAvoiderOptions options;
    options.set_near_field_range(2.0);
    options.set_reaction_time_ms(30.0);
    options.set_safety_margin(0.3);
    options.set_fov_rad(1.57);
    options.set_candidate_count(16);
    return options;
}

}  // namespace

int main()
{
    using navigation::common::Twist2D;
    using navigation::safety::ReflexPolicyArbiter;

    ReflexPolicyArbiter arbiter;
    const Twist2D desired = navigation::common::MakeTwist2D(1.0, 0.0, 0.0);

    const auto clear = arbiter.Arbitrate(desired, Obstacle(3.0), Options(), 0.2);
    if (clear.intervened || clear.velocity.x() != desired.x())
    {
        std::cerr << "clear path intervened\n";
        return 1;
    }

    const auto close = arbiter.Arbitrate(desired, Obstacle(0.352, -5.0), Options(), 0.2);
    if (!close.intervened || close.risk < 0.5)
    {
        std::cerr << "close obstacle not handled\n";
        return 1;
    }
    if (close.velocity.x() >= desired.x())
    {
        std::cerr << "reflex did not slow down\n";
        return 1;
    }
    return 0;
}
