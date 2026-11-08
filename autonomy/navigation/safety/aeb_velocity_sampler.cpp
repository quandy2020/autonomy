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

#include "navigation/safety/aeb_velocity_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace safety
{

std::optional<common::Twist2D> AebVelocitySampler::Sample(
    const common::Twist2D& desired, const ReflexObservation& observation,
    const common::PlatformProfile& profile, const CbfFilter& filter,
    std::size_t sample_count) const
{
    if (sample_count == 0 || !common::IsValid(profile))
    {
        return std::nullopt;
    }
    common::Twist2D zero;
    if (filter.IsSafe(zero, observation))
    {
        const double max_linear = std::max(0.05, profile.max_linear_mps);
        const double step = max_linear / static_cast<double>(sample_count);
        constexpr double kAngularFractions[] = {0.0, -0.25, 0.25, -0.5, 0.5};
        for (double speed = max_linear; speed > 0.0; speed -= step)
        {
            for (double fraction : kAngularFractions)
            {
                common::Twist2D candidate;
                candidate.set_x(std::min(speed, static_cast<double>(desired.x())));
                candidate.set_y(0.0);
                candidate.set_theta(fraction * profile.max_angular_radps);
                if (filter.IsSafe(candidate, observation))
                {
                    return candidate;
                }
            }
        }
    }
    if (filter.IsSafe(zero, observation))
    {
        return zero;
    }
    return std::nullopt;
}

}  // namespace safety
}  // namespace navigation
