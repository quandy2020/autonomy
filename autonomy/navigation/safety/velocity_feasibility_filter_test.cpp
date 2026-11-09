/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <deque>
#include <iostream>

#include "navigation/safety/velocity_feasibility_filter.hpp"

namespace {

// Close the loop through an actuator and an odometry transport delay. Check
// settled windows, so one zero crossing cannot hide a persistent oscillation.
bool Converges(int delay_steps) {
    using navigation::common::MakeTwist2D;
    using navigation::common::Twist2D;
    navigation::common::PlatformProfile profile;
    navigation::safety::VelocityFeasibilityFilter filter;
    if (!filter.Configure(profile))
        return false;
    constexpr double dt = 0.02;
    std::deque<Twist2D> odometry(delay_steps + 1);
    Twist2D previous_acceleration;
    bool has_acceleration = false;
    const Twist2D moving = MakeTwist2D(0.4, 0.0, 0.8);
    // Hold a speed, reverse it, then stop. A delayed measurement must not
    // amplify the acceleration or keep the actuator moving after settling.
    for (int phase = 0; phase < 3; ++phase) {
        const Twist2D target = phase == 0   ? moving
                               : phase == 1 ? MakeTwist2D(-0.4, 0.0, -0.8)
                                            : Twist2D{};
        for (int step = 0; step < 500; ++step) {
            const Twist2D measured = odometry.front();
            Twist2D command;
            if (!filter.Filter(target, measured, dt, &command))
                return false;
            const Twist2D acceleration =
                MakeTwist2D((command.x() - measured.x()) / dt, 0.0,
                            (command.theta() - measured.theta()) / dt);
            if (std::abs(command.x()) > profile.max_linear_mps + 1e-6 ||
                std::abs(command.theta()) > profile.max_angular_radps + 1e-6 ||
                std::abs(acceleration.x()) >
                    profile.max_acceleration_mps2 + 1e-5 ||
                std::abs(acceleration.theta()) >
                    profile.max_acceleration_mps2 / profile.footprint_radius +
                        1e-5 ||
                (has_acceleration &&
                 (std::abs(acceleration.x() - previous_acceleration.x()) >
                      profile.max_jerk_mps3 * dt + 1e-4 ||
                  std::abs(acceleration.theta() -
                           previous_acceleration.theta()) >
                      profile.max_jerk_mps3 * dt + 1e-4))) {
                std::cerr << "speed/acceleration/jerk bound violated\n";
                return false;
            }
            if (step >= 400 &&
                (std::abs(command.x() - target.x()) > 1e-3 ||
                 std::abs(command.theta() - target.theta()) > 1e-3)) {
                std::cerr << "velocity did not converge: delay=" << delay_steps
                          << " phase=" << phase << " step=" << step
                          << " vx=" << command.x() << " wz=" << command.theta()
                          << "\n";
                return false;
            }
            previous_acceleration = acceleration;
            has_acceleration = true;
            odometry.push_back(command);
            odometry.pop_front();
        }
    }
    return true;
}

}  // namespace

int main() {
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
    if (!filter.Configure(profile)) {
        std::cerr << "configure failed\n";
        return 1;
    }
    Twist2D filtered;
    if (!filter.Filter(navigation::common::MakeTwist2D(0.5, 0.0, 0.0),
                       navigation::common::MakeTwist2D(0.0, 0.0, 0.0), 0.1,
                       &filtered)) {
        std::cerr << "filter failed\n";
        return 1;
    }
    if (std::abs(filtered.x() - 0.05) > 1e-9) {
        std::cerr << "acceleration limit failed: " << filtered.x() << "\n";
        return 1;
    }
    if (filter.Filter(navigation::common::MakeTwist2D(0.5, 0.0, 0.0),
                      navigation::common::MakeTwist2D(0.0, 0.0, 0.0), 0.0,
                      &filtered)) {
        std::cerr << "zero dt accepted\n";
        return 1;
    }
    for (int delay : {0, 1, 3}) {
        if (!Converges(delay))
            return 1;
    }
    return 0;
}
