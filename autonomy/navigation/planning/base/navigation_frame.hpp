/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file navigation_frame.hpp
 * @brief Mutable per-tick frame passed through scenario stages.
 */

#pragma once

#include <cstdint>

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace planning
{

/** @brief Everything a stage may read or update during one tick. */
struct NavigationFrame
{
    /** @brief Observation for this tick; borrowed. */
    const observation::Observation* observation{nullptr};
    /** @brief Immutable memory context; borrowed. */
    const memory::ContextView* context{nullptr};
    /** @brief Platform limits; borrowed, may be null. */
    const common::PlatformProfile* platform{nullptr};
    /** @brief Near-field obstacles; borrowed, may be null. */
    const ReflexObservation* reflex{nullptr};
    /** @brief Goal being executed. */
    GoalSpec goal;
    /** @brief Current map-frame pose. */
    common::Pose2D pose;
    /** @brief Measured velocity. */
    common::Twist2D velocity;
    /** @brief Candidate command modified by stages/rules. */
    common::Twist2D command;
    /** @brief Policy intent snapshot. */
    NavigationIntent intent;
    /** @brief Monotonic tick time in nanoseconds. */
    std::uint64_t now_ns{0};
    /** @brief Session epoch. */
    std::uint32_t epoch{0};
    /** @brief Tick period in seconds. */
    double dt_s{0.0};
    /** @brief Set when a stage/rule degraded the frame. */
    bool degraded{false};
};

}  // namespace planning
}  // namespace navigation
