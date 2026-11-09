/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <cmath>
#include <iostream>

#include "navigation/safety/safety_shield_implementation.hpp"

namespace {

navigation::common::PlatformProfile Profile() {
    navigation::common::PlatformProfile profile;
    profile.kinematic_type = navigation::common::KinematicType::kDifferential;
    profile.max_linear_mps = 0.5;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 1.0;
    profile.max_jerk_mps3 = 5.0;
    profile.max_lateral_acceleration_mps2 = 0.5;
    profile.footprint_radius = 0.3;
    return profile;
}

navigation::config::SafetyChainOptions Chain() {
    navigation::config::SafetyChainOptions chain;
    chain.set_reflex_priority_ratio(0.2);
    chain.set_max_acceleration_mps2(1.0);
    chain.set_max_jerk_mps3(5.0);
    chain.set_ttc_reaction_time_s(0.15);
    chain.set_ttc_safety_margin_s(0.1);
    chain.set_cbf_alpha(2.0);
    chain.set_watchdog_deadline_ms(50);
    chain.set_aeb_sample_count(9);
    chain.set_min_clearance(0.05);
    chain.set_max_hold_ms(200);
    return chain;
}

navigation::plugins::ReactiveAvoiderOptions Reflex() {
    navigation::plugins::ReactiveAvoiderOptions options;
    options.set_near_field_range(2.0);
    options.set_reaction_time_ms(30.0);
    options.set_safety_margin(0.3);
    options.set_fov_rad(1.57);
    options.set_candidate_count(16);
    return options;
}

navigation::ReflexObservation Observation(std::int64_t stamp_ns) {
    navigation::ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(stamp_ns);
    observation.set_coverage_radius(2.0);
    return observation;
}

}  // namespace

int main() {
    using navigation::common::Twist2D;
    using navigation::safety::SafetyShieldImplementation;

    SafetyShieldImplementation shield;
    if (!shield.Configure(Profile(), Chain(), Reflex())) {
        std::cerr << "configure failed\n";
        return 1;
    }

    const std::int64_t now =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    Twist2D measured = navigation::common::MakeTwist2D(0.3, 0.0, 0.0);
    const auto normal =
        shield.Process(navigation::common::MakeTwist2D(0.3, 0.0, 0.0),
                       Observation(now), measured, now, 0.1);
    if (normal.emergency) {
        std::cerr << "normal path stopped: " << normal.reason << "\n";
        return 1;
    }
    if (!std::isfinite(normal.command.x())) {
        std::cerr << "non-finite command\n";
        return 1;
    }

    const auto invalid =
        shield.Process(navigation::common::MakeTwist2D(std::nan(""), 0.0, 0.0),
                       Observation(now), measured, now, 0.1);
    if (!invalid.emergency || invalid.command.x() != 0.0) {
        std::cerr << "invalid input did not stop\n";
        return 1;
    }

    const auto stale =
        shield.Process(navigation::common::MakeTwist2D(0.3, 0.0, 0.0),
                       Observation(now - 1000000000LL), measured, now, 0.1);
    if (!stale.emergency) {
        std::cerr << "stale input did not stop\n";
        return 1;
    }

    auto blocked = Observation(now);
    auto* obstacle = blocked.add_obstacles();
    obstacle->set_x(0.1);
    obstacle->set_radius(0.1);
    const auto collision =
        shield.Process(navigation::common::MakeTwist2D(0.3, 0.0, 0.0), blocked,
                       measured, now, 0.1);
    if (!collision.emergency) {
        std::cerr << "collision path did not stop\n";
        return 1;
    }
    for (const bool obstacle_stop : {false, true}) {
        shield.Reset();
        const auto braking =
            shield.Process(Twist2D{}, Observation(now), measured, now, 0.02);
        if (braking.emergency || braking.command.x() >= measured.x()) {
            std::cerr << "could not establish deceleration history\n";
            return 1;
        }
        const auto emergency = shield.Process(
            Twist2D{},
            obstacle_stop ? blocked : Observation(now - 1000000000LL), measured,
            now, 0.02);
        const auto recovered =
            shield.Process(navigation::common::MakeTwist2D(0.3, 0.0, 0.0),
                           Observation(now), Twist2D{}, now, 0.02);
        if (!emergency.emergency || recovered.emergency ||
            recovered.command.x() <= 0.0 ||
            recovered.command.x() >
                Profile().max_acceleration_mps2 * 0.02 + 1e-5) {
            std::cerr << "emergency recovery retained old acceleration\n";
            return 1;
        }
    }
    shield.Reset();
    auto buffer = Observation(now);
    buffer.add_obstacles()->set_x(0.45 - 1e-7);
    buffer.mutable_obstacles(0)->set_radius(0.1);
    Twist2D turning;
    for (int tick = 0; tick < 80; ++tick) {
        buffer.set_stamp_ns(now + tick * 20000000LL);
        const auto step =
            shield.Process(navigation::common::MakeTwist2D(0.0, 0.0, 0.6),
                           buffer, turning, now + tick * 20000000LL, 0.02);
        if (step.emergency || std::abs(step.command.x()) > 1e-6) {
            std::cerr << "blocked forward motion prevented recovery turn\n";
            return 1;
        }
        turning = step.command;
    }
    if (turning.theta() < 0.5)
        return 1;
    return 0;
}
