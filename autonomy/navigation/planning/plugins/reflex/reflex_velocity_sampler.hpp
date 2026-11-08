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
 * @file reflex_velocity_sampler.hpp
 * @brief Velocity-space sampling helper for near-field reflex.
 */

#pragma once

#include <optional>

#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{

class ReflexVelocitySampler
{
public:
    /** @brief Pick a lateral escape yaw sign for an obstacle offset. */
    [[nodiscard]] static double AvoidanceYaw(double lateral, double max_yaw_rate);

    /** @brief Sample a slow escape velocity opposite the nearest obstacle. */
    [[nodiscard]] std::optional<common::Twist2D> Sample(
        const ReflexObservation& observation, const common::Twist2D& desired,
        double max_linear_mps) const;
};

}  // namespace planning
}  // namespace navigation
