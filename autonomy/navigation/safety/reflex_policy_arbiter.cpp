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

#include "navigation/safety/reflex_policy_arbiter.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{
namespace safety
{

ReflexCommand ReflexPolicyArbiter::Arbitrate(
    const common::Twist2D& desired, const ReflexObservation& observation,
    const plugins::ReactiveAvoiderOptions& options, double priority_ratio) const
{
    ReflexCommand result;
    result.velocity = desired;
    if (!observation.valid() || !common::IsFinite(desired))
    {
        return result;
    }

    const double reaction_s = std::max(0.0, options.reaction_time_ms() / 1000.0);
    const double threshold = std::max(0.05, reaction_s + 0.05);
    const auto ttc = ttc_.MinTtc(observation, desired, 0.3);
    if (!ttc.has_value() || *ttc >= threshold)
    {
        return result;
    }

    const double risk = autonomy::common::Clamp(1.0 - *ttc / threshold, 0.0, 1.0);
    const double weight = std::max(autonomy::common::Clamp(priority_ratio, 0.0, 1.0), risk);

    double nearest_distance = std::numeric_limits<double>::infinity();
    double avoid_y = 0.0;
    for (const auto& obstacle : observation.obstacles())
    {
        const double distance = std::hypot(obstacle.x(), obstacle.y());
        if (distance > options.near_field_range() || distance >= nearest_distance)
        {
            continue;
        }
        const double bearing = std::atan2(obstacle.y(), obstacle.x());
        if (std::abs(bearing) > 0.5 * options.fov_rad())
        {
            continue;
        }
        nearest_distance = distance;
        avoid_y = obstacle.y();
    }

    common::Twist2D reflex = desired;
    reflex.set_x(desired.x() * (1.0 - 0.8 * risk));
    reflex.set_y(desired.y() * (1.0 - 0.8 * risk));
    const double steer_sign = avoid_y >= 0.0 ? -1.0 : 1.0;
    reflex.set_theta(autonomy::common::Clamp(desired.theta() + steer_sign * risk * 0.8, -1.0, 1.0));

    result.velocity.set_x((1.0 - weight) * desired.x() + weight * reflex.x());
    result.velocity.set_y((1.0 - weight) * desired.y() + weight * reflex.y());
    result.velocity.set_theta((1.0 - weight) * desired.theta() + weight * reflex.theta());
    result.intervened = weight >= 0.8;
    result.risk = risk;
    return result;
}

}  // namespace safety
}  // namespace navigation
