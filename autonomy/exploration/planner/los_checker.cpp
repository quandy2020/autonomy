/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/exploration/planner/los_checker.hpp"

#include <cmath>
#include <vector>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/line_iterator.hpp"

namespace autonomy {
namespace exploration {

bool HasLineOfSight(const PlanningEnv& env, double x0, double y0, double x1,
                    double y1, bool stop_at_unknown)
{
    // Prefer inflated map so robot footprint clears obstacles.
    const auto& costmap = env.inflated_costmap();
    unsigned int mx0 = 0;
    unsigned int my0 = 0;
    unsigned int mx1 = 0;
    unsigned int my1 = 0;
    if (!costmap.worldToMap(x0, y0, mx0, my0) ||
        !costmap.worldToMap(x1, y1, mx1, my1)) {
        return false;
    }
    if (mx0 == mx1 && my0 == my1) {
        return true;
    }

    map::costmap_2d::utils::LineIterator line(
        static_cast<int>(mx0), static_cast<int>(my0), static_cast<int>(mx1),
        static_cast<int>(my1));
    // Skip the start cell; test intermediate cells only (allow endpoint).
    if (line.isValid()) {
        line.advance();
    }
    while (line.isValid()) {
        const unsigned int mx = static_cast<unsigned int>(line.getX());
        const unsigned int my = static_cast<unsigned int>(line.getY());
        const bool is_end = (mx == mx1 && my == my1);
        if (is_end) {
            break;
        }
        const unsigned char cost = costmap.getCost(mx, my);
        if (cost >= map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;
        }
        if (stop_at_unknown && cost == map::costmap_2d::NO_INFORMATION) {
            return false;
        }
        line.advance();
    }
    return true;
}

double PathLengthXy(
    const std::vector<commsgs::geometry_msgs::Point>& points)
{
    if (points.size() < 2) {
        return 0.0;
    }
    double len = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        const double dx = points[i].x - points[i - 1].x;
        const double dy = points[i].y - points[i - 1].y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

}  // namespace exploration
}  // namespace autonomy
