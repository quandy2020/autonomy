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

#include "navigation/planning/tasks/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{
namespace planning
{
namespace
{

bool Contains(const Region& region, double x, double y)
{
    bool inside = false;
    const int count = region.polygon_size();
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

}  // namespace

CoveragePlanner::CoveragePlanner(CoverageOptions options) : options_(options)
{
    if (options_.spacing <= 0.0)
    {
        options_.spacing = 0.5;
    }
}

bool CoveragePlanner::Plan(const Region& region, std::vector<Waypoint>* waypoints) const
{
    if (waypoints == nullptr || region.polygon_size() < 3)
    {
        return false;
    }
    waypoints->clear();
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& point : region.polygon())
    {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }

    bool left_to_right = true;
    for (double y = min_y; y <= max_y; y += options_.spacing)
    {
        std::vector<double> xs;
        for (double x = min_x; x <= max_x; x += options_.spacing)
        {
            if (Contains(region, x, y))
            {
                xs.push_back(x);
            }
        }
        if (xs.empty())
        {
            continue;
        }
        if (!left_to_right)
        {
            std::reverse(xs.begin(), xs.end());
        }
        for (double x : xs)
        {
            Waypoint waypoint;
            waypoint.pose.set_x(x);
            waypoint.pose.set_y(y);
            waypoint.tolerance = options_.tolerance;
            waypoints->push_back(waypoint);
        }
        left_to_right = !left_to_right;
    }
    return !waypoints->empty();
}

void CoveragePlanner::Reset()
{
}

}  // namespace planning
}  // namespace navigation
