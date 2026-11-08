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

#include "navigation/planning/rules/pedestrian_rule.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace planning
{

common::Status PedestrianRule::Apply(NavigationFrame& frame, common::Twist2D* command)
{
    if (command == nullptr)
    {
        return common::Status::InvalidArgument("command must not be null");
    }
    if (frame.reflex == nullptr || !frame.reflex->valid())
    {
        return common::Status{};
    }
    const double radius = frame.platform != nullptr ? frame.platform->footprint_radius : 0.3;
    double clearance = 1e9;
    for (const auto& obstacle : frame.reflex->obstacles())
    {
        clearance = std::min(clearance,
                             std::hypot(obstacle.x(), obstacle.y()) - radius -
                                 obstacle.radius());
    }
    if (clearance < 0.4)
    {
        *command = common::Twist2D{};
        return common::Status::Unavailable("pedestrian too close");
    }
    if (clearance < 1.0)
    {
        const double scale = std::clamp((clearance - 0.4) / 0.6, 0.0, 1.0);
        command->set_x(command->x() * (scale));
        command->set_y(command->y() * (scale));
    }
    return common::Status{};
}

void PedestrianRule::Reset()
{
}

std::string_view PedestrianRule::Name() const
{
    return "pedestrian";
}

}  // namespace planning
}  // namespace navigation
