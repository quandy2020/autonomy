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

#include "navigation/planning/rules/forbidden_zone_rule.hpp"

#include <utility>

namespace navigation
{
namespace planning
{

void ForbiddenZoneRule::SetZones(std::vector<Region> zones)
{
    zones_ = std::move(zones);
}

bool ForbiddenZoneRule::Contains(const Region& region, double x, double y)
{
    const int count = region.polygon_size();
    if (count < 3)
    {
        return false;
    }
    bool inside = false;
    for (int i = 0, j = count - 1; i < count; j = i++)
    {
        const auto& current = region.polygon(i);
        const auto& previous = region.polygon(j);
        const bool intersects =
            ((current.y() > y) != (previous.y() > y)) &&
            (x < (previous.x() - current.x()) * (y - current.y()) /
                           (previous.y() - current.y()) +
                       current.x());
        if (intersects)
        {
            inside = !inside;
        }
    }
    return inside;
}

common::Status ForbiddenZoneRule::Apply(NavigationFrame& frame, common::Twist2D* command)
{
    if (command == nullptr)
    {
        return common::Status::InvalidArgument("command must not be null");
    }
    for (const auto& zone : zones_)
    {
        if (Contains(zone, frame.pose.x(), frame.pose.y()))
        {
            *command = common::Twist2D{};
            return common::Status::Unavailable("inside forbidden zone");
        }
    }
    return common::Status{};
}

void ForbiddenZoneRule::Reset()
{
}

std::string_view ForbiddenZoneRule::Name() const
{
    return "forbidden_zone";
}

}  // namespace planning
}  // namespace navigation
