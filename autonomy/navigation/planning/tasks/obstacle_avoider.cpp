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

#include "navigation/planning/tasks/obstacle_avoider.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace planning
{

common::Status ObstacleAvoider::Execute(NavigationFrame& frame)
{
    if (frame.reflex == nullptr || !frame.reflex->valid())
    {
        return common::Status{};
    }
    const double radius = frame.platform != nullptr ? frame.platform->footprint_radius : 0.3;
    double nearest = 1e9;
    double avoid_y = 0.0;
    for (const auto& obstacle : frame.reflex->obstacles())
    {
        const double distance = std::hypot(obstacle.x(), obstacle.y());
        if (distance < nearest)
        {
            nearest = distance;
            avoid_y = obstacle.y();
        }
    }
    const double clearance = nearest - radius;
    if (clearance < 0.15)
    {
        frame.command = common::Twist2D{};
        return common::Status::Unavailable("obstacle too close");
    }
    if (clearance < 0.6)
    {
        const double certainty = autonomy::common::Clamp(1.0 - clearance / 0.6, 0.0, 1.0);
        frame.command.set_x(frame.command.x() * ((1.0 - 0.7 * certainty)));
        frame.command.set_theta(frame.command.theta() + ((avoid_y >= 0.0 ? -1.0 : 1.0) * certainty));
        if (frame.platform != nullptr)
        {
            frame.command.set_theta(autonomy::common::Clamp(static_cast<double>(frame.command.theta()),
                              -frame.platform->max_angular_radps,
                              frame.platform->max_angular_radps));
        }
    }
    return common::Status{};
}

void ObstacleAvoider::Reset()
{
}

std::string_view ObstacleAvoider::Name() const
{
    return "obstacle_avoider";
}

}  // namespace planning
}  // namespace navigation
