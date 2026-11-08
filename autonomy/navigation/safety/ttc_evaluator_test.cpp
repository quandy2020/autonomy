/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/safety/ttc_evaluator.hpp"

int main()
{
    using navigation::ReflexObservation;
    using navigation::common::Twist2D;
    using navigation::safety::TtcEvaluator;

    ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(1);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(2.0);
    obstacle->set_vx_mps(-1.0);
    obstacle->set_radius(0.2);

    TtcEvaluator evaluator;
    const Twist2D velocity = navigation::common::MakeTwist2D(1.0, 0.0, 0.0);
    const auto ttc = evaluator.MinTtc(observation, velocity, 0.3);
    if (!ttc.has_value() || std::abs(*ttc - 0.75) > 1e-6)
    {
        std::cerr << "ttc failed: " << (ttc ? *ttc : -1.0) << "\n";
        return 1;
    }
    if (evaluator.MinClearance(observation, 0.3) < 1.4)
    {
        std::cerr << "clearance failed\n";
        return 1;
    }

    ReflexObservation empty;
    empty.set_valid(true);
    if (evaluator.MinTtc(empty, velocity, 0.3).has_value())
    {
        std::cerr << "empty ttc reported\n";
        return 1;
    }
    return 0;
}
