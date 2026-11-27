/* Copyright 2026 The Openbot Authors. */
/** @file momentum_tracker_test.cpp
 * @brief Regressions for measured-motion relocation momentum hysteresis.
 */
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "autonomy/task/exploration/routing/momentum_tracker.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace n = autonomy::navigation;

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

e::MomentumSampleIdentity Identity(std::uint64_t session = 1,
                                   std::uint64_t epoch = 2,
                                   std::uint64_t localization = 3) {
    return {session, epoch, localization, "map"};
}

n::Pose2d Pose(double x, double y = 0) {
    return {x, y, 0};
}

void StrictMeasuredHysteresis() {
    e::MomentumTracker tracker({0.5, 1, 1});
    Require(!tracker.Update(Pose(0), 1, Identity()).active,
            "baseline unexpectedly activated momentum");
    Require(!tracker.Update(Pose(1), 2, Identity()).active,
            "first measured direction unexpectedly activated momentum");
    auto state = tracker.Update(Pose(-1), 3, Identity());
    Require(!state.active && state.reversal_count == 1,
            "activation threshold was not strict");
    state = tracker.Update(Pose(1), 4, Identity());
    Require(state.active && state.reversal_count == 2 &&
                state.preferred_direction &&
                std::abs(state.preferred_direction->x() - 1) < 1e-12,
            "measured reversals did not activate preferred direction");
    state = tracker.Update(Pose(2), 5, Identity());
    Require(state.active && state.stable_count == 1,
            "release threshold was not strict");
    state = tracker.Update(Pose(3), 6, Identity());
    Require(
        !state.active && state.stable_count == 2 && !state.preferred_direction,
        "stable measured motion did not release momentum");
}

void SamplingDistanceAndTimestampCursor() {
    e::MomentumTracker tracker({1.0, 0, 0});
    tracker.Update(Pose(0), 1, Identity());
    tracker.Update(Pose(0.5), 2, Identity());
    Require(!tracker.state().preferred_direction,
            "sub-threshold displacement created a direction");
    tracker.Update(Pose(2), 1.5, Identity());
    Require(!tracker.state().preferred_direction,
            "timestamp rollback retained momentum state");
    const auto state = tracker.Update(Pose(0), 3, Identity());
    Require(state.reversal_count == 0,
            "time discontinuity did not establish a clean baseline");
}

void IdentityDiscontinuitiesReset() {
    e::MomentumTracker tracker({0.5, 0, 10});
    tracker.Update(Pose(0), 1, Identity());
    tracker.Update(Pose(1), 2, Identity());
    auto state = tracker.Update(Pose(-1), 3, Identity());
    Require(state.active, "fixture did not activate momentum");
    state = tracker.Update(Pose(-2), 4, Identity(1, 9, 3));
    Require(!state.active && state.reversal_count == 0 &&
                !state.preferred_direction,
            "map epoch discontinuity retained momentum");
    tracker.Update(Pose(-3), 5, Identity(1, 9, 3));
    state = tracker.Update(Pose(-1), 6, Identity(1, 9, 4));
    Require(!state.active && state.reversal_count == 0,
            "localization discontinuity retained momentum");
    state = tracker.Update(Pose(0), 7, Identity(2, 9, 4));
    Require(!state.active && state.reversal_count == 0,
            "session discontinuity retained momentum");
}

void InvalidInputFailsClosed() {
    bool threw = false;
    try {
        e::MomentumTracker invalid({0, 1, 1});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(threw, "zero sampling distance was accepted");

    e::MomentumTracker tracker({0.5, 0, 1});
    tracker.Update(Pose(0), 1, Identity());
    tracker.Update(Pose(1), 2, Identity());
    tracker.Update(Pose(-1), 3, Identity());
    auto invalid_pose = Pose(0);
    invalid_pose.x = std::nan("");
    const auto state = tracker.Update(invalid_pose, 4, Identity());
    Require(!state.active && !state.preferred_direction,
            "invalid localization failed to clear momentum");
}

}  // namespace

int main() {
    try {
        StrictMeasuredHysteresis();
        SamplingDistanceAndTimestampCursor();
        IdentityDiscontinuitiesReset();
        InvalidInputFailsClosed();
        std::cout << "momentum tracker tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
