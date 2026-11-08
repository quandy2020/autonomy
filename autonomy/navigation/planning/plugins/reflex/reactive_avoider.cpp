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

#include "navigation/planning/plugins/reflex/reactive_avoider.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navigation
{
namespace planning
{

ReactiveAvoider::ReactiveAvoider(plugins::ReactiveAvoiderOptions options)
    : options_(std::move(options))
{
}

ReflexCommand ReactiveAvoider::Compute(const ReflexObservation& observation,
                                       const common::Twist2D& desired, double dt_s)
{
    (void)dt_s;
    ReflexCommand result;
    result.velocity = desired;
    if (!observation.valid() || !common::IsFinite(desired))
    {
        return result;
    }
    const double range = options_.near_field_range() > 0.0
                             ? options_.near_field_range()
                             : 2.0;
    double nearest = range;
    double lateral = 0.0;
    bool found = false;
    for (const auto& obstacle : observation.obstacles())
    {
        const double distance = std::hypot(obstacle.x(), obstacle.y());
        const double bearing = std::atan2(obstacle.y(), obstacle.x());
        if (distance > range || std::abs(bearing) > 0.5 * options_.fov_rad())
        {
            continue;
        }
        if (distance < nearest)
        {
            nearest = distance;
            lateral = obstacle.y();
            found = true;
        }
    }
    if (!found)
    {
        return result;
    }
    const double clearance = nearest - options_.safety_margin();
    if (clearance <= 0.15)
    {
        result.velocity = common::Twist2D{};
        result.intervened = true;
        result.risk = 1.0;
        return result;
    }
    const double risk = autonomy::common::Clamp(1.0 - clearance / range, 0.0, 1.0);
    result.velocity.set_x(desired.x() * (1.0 - 0.8 * risk));
    result.velocity.set_y(desired.y() * (1.0 - 0.8 * risk));
    const double steer = (lateral >= 0.0 ? -1.0 : 1.0) * risk * 0.8;
    result.velocity.set_theta(autonomy::common::Clamp(desired.theta() + steer, -1.0, 1.0));
    result.intervened = risk >= 0.8;
    result.risk = risk;
    return result;
}

void ReactiveAvoider::Reset()
{
}

}  // namespace planning
}  // namespace navigation
