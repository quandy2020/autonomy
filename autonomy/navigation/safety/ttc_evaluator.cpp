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

#include "navigation/safety/ttc_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{
namespace safety
{

std::optional<double> TtcEvaluator::MinTtc(const ReflexObservation& observation,
                                           const common::Twist2D& robot_velocity,
                                           double robot_radius) const
{
    if (!observation.valid())
    {
        return std::nullopt;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& obstacle : observation.obstacles())
    {
        const double px = obstacle.x();
        const double py = obstacle.y();
        const double rvx = obstacle.vx_mps() - robot_velocity.x();
        const double rvy = obstacle.vy_mps() - robot_velocity.y();
        const double speed_squared = rvx * rvx + rvy * rvy;
        if (!std::isfinite(px) || !std::isfinite(py) || speed_squared < 1e-12)
        {
            continue;
        }
        const double collision_radius = robot_radius + obstacle.radius();
        const double c = px * px + py * py - collision_radius * collision_radius;
        if (c <= 0.0)
        {
            // Already inside the collision disc.
            return 0.0;
        }
        const double b = 2.0 * (px * rvx + py * rvy);
        const double discriminant = b * b - 4.0 * speed_squared * c;
        if (discriminant < 0.0)
        {
            continue;
        }
        const double t = (-b - std::sqrt(discriminant)) / (2.0 * speed_squared);
        if (t > 0.0)
        {
            best = std::min(best, t);
        }
    }
    if (!std::isfinite(best))
    {
        return std::nullopt;
    }
    return best;
}

double TtcEvaluator::MinClearance(const ReflexObservation& observation,
                                  double robot_radius) const
{
    if (!observation.valid())
    {
        return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& obstacle : observation.obstacles())
    {
        const double distance = std::hypot(obstacle.x(), obstacle.y());
        best = std::min(best, distance - robot_radius - obstacle.radius());
    }
    return best;
}

}  // namespace safety
}  // namespace navigation
