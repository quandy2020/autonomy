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

#include "navigation/planning/plugins/reflex/reflex_velocity_sampler.hpp"
#include "autonomy/common/math/math.hpp"

#include <cmath>

namespace navigation
{
namespace planning
{

double ReflexVelocitySampler::AvoidanceYaw(double lateral, double max_yaw_rate)
{
    const double sign = lateral >= 0.0 ? -1.0 : 1.0;
    return sign * std::abs(max_yaw_rate);
}

std::optional<common::Twist2D> ReflexVelocitySampler::Sample(
    const ReflexObservation& observation, const common::Twist2D& desired,
    double max_linear_mps) const
{
    if (!observation.valid() || !common::IsFinite(desired))
    {
        return std::nullopt;
    }
    double nearest = 1e9;
    double lateral = 0.0;
    for (const auto& obstacle : observation.obstacles())
    {
        const double distance = std::hypot(obstacle.x(), obstacle.y());
        if (distance < nearest)
        {
            nearest = distance;
            lateral = obstacle.y();
        }
    }
    if (nearest > 1e8)
    {
        return desired;
    }
    common::Twist2D command;
    command.set_x(autonomy::common::Clamp(nearest * 0.3, 0.0, max_linear_mps));
    command.set_theta(AvoidanceYaw(lateral, 0.8));
    return command;
}

}  // namespace planning
}  // namespace navigation
