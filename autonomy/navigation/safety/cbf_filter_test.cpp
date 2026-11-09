/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/platform_registry.hpp"
#include "navigation/safety/cbf_filter.hpp"

namespace {

navigation::common::PlatformProfile Profile() {
    navigation::common::PlatformProfile profile;
    profile.kinematic_type = navigation::common::KinematicType::kDifferential;
    profile.max_linear_mps = 2.0;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 1.0;
    profile.max_jerk_mps3 = 5.0;
    profile.footprint_radius = 0.3;
    return profile;
}

navigation::ReflexObservation Obstacle(double x) {
    navigation::ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(1);
    auto* obstacle = observation.add_obstacles();
    obstacle->set_x(x);
    obstacle->set_radius(0.2);
    return observation;
}

}  // namespace

int main() {
    using navigation::common::Twist2D;
    using navigation::safety::CbfFilter;

    CbfFilter filter;
    if (!filter.Configure(Profile(), 0.1, 0.1)) {
        std::cerr << "configure failed\n";
        return 1;
    }
    const auto observation = Obstacle(1.0);
    if (filter.IsSafe(navigation::common::MakeTwist2D(0.5, 0.0, 0.0),
                      observation)) {
        std::cerr << "unsafe velocity accepted\n";
        return 1;
    }
    const auto projected = filter.Project(
        navigation::common::MakeTwist2D(1.0, 0.0, 0.0), observation);
    if (!projected.has_value() || projected->x() > 0.05) {
        std::cerr << "projection failed\n";
        return 1;
    }
    if (!filter.IsSafe(*projected, observation)) {
        std::cerr << "projection unsafe\n";
        return 1;
    }
    // Inside the clearance buffer but outside the physical collision disc:
    // rotation must stay feasible so the robot can face away from the obstacle.
    const auto inside = Obstacle(0.55);
    if (!filter.IsSafe(navigation::common::MakeTwist2D(0.0, 0.0, 0.5),
                       inside)) {
        std::cerr << "inside barrier: rotation rejected\n";
        return 1;
    }
    const auto escape =
        filter.Project(navigation::common::MakeTwist2D(0.5, 0.0, 0.5), inside);
    if (!escape.has_value() || escape->x() > 0.05) {
        std::cerr << "inside barrier: escape projection failed\n";
        return 1;
    }
    if (!filter.IsSafe(*escape, inside)) {
        std::cerr << "inside barrier: escape unsafe\n";
        return 1;
    }
    const auto clear = Obstacle(-3.0);
    if (!filter.IsSafe(navigation::common::MakeTwist2D(1.0, 0.0, 0.0), clear)) {
        std::cerr << "free space rejected\n";
        return 1;
    }
    // Braking lands infinitesimally inside the configured clearance buffer.
    // A circular footprint can rotate without approaching the obstacle.
    const auto boundary = Obstacle(0.6 - 1e-7);
    const auto turn = navigation::common::MakeTwist2D(0.0, 0.0, 0.6);
    const auto forward = navigation::common::MakeTwist2D(0.1, 0.0, 0.0);
    if (!filter.IsSafe(turn, boundary) || filter.IsSafe(forward, boundary) ||
        !filter.IsSafe(navigation::common::MakeTwist2D(-0.1, 0.0, 0.0),
                       boundary)) {
        std::cerr << "clearance buffer rejected escape or allowed approach\n";
        return 1;
    }
    // Physical overlap must still allow escape: forward is rejected, reverse
    // and rotation stay feasible so the robot can back out of the obstacle.
    auto actual_collision = Obstacle(0.5 - 1e-7);
    if (filter.IsSafe(forward, actual_collision)) {
        std::cerr << "physical overlap allowed approach\n";
        return 1;
    }
    if (!filter.IsSafe(turn, actual_collision) ||
        !filter.IsSafe(navigation::common::MakeTwist2D(-0.1, 0.0, 0.0),
                       actual_collision)) {
        std::cerr << "physical overlap blocked escape\n";
        return 1;
    }
    const auto overlap_escape = filter.Project(
        navigation::common::MakeTwist2D(0.3, 0.0, 0.4), actual_collision);
    if (!overlap_escape.has_value() || overlap_escape->x() > 0.0) {
        std::cerr << "physical overlap projection did not escape\n";
        return 1;
    }
    return 0;
}
