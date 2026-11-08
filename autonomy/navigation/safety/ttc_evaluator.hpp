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
 * @file ttc_evaluator.hpp
 * @brief Time-to-collision over a planar obstacle set.
 */

#pragma once

#include <optional>

#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace safety
{

class TtcEvaluator
{
public:
    /**
     * @brief Minimum time to collision.
     *
     * @param[in] observation Near-field obstacles.
     * @param[in] robot_velocity Current planar velocity.
     * @param[in] robot_radius Collision radius in metres.
     * @return Positive TTC in seconds, or nullopt when no collision course.
     */
    [[nodiscard]] std::optional<double> MinTtc(const ReflexObservation& observation,
                                               const common::Twist2D& robot_velocity,
                                               double robot_radius) const;

    /**
     * @brief Minimum distance to any obstacle surface.
     *
     * @param[in] observation Near-field obstacles.
     * @param[in] robot_radius Robot collision radius in metres.
     * @return Clearance in metres, or +inf for no obstacles.
     */
    [[nodiscard]] double MinClearance(const ReflexObservation& observation,
                                      double robot_radius) const;
};

}  // namespace safety
}  // namespace navigation
