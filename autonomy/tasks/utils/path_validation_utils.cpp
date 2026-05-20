/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/tasks/utils/path_validation_utils.hpp"

#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy {
namespace tasks {
namespace utils {

bool IsPathValidOnCostmap(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    const commsgs::planning_msgs::Path& path, unsigned int max_cost,
    bool consider_unknown_as_obstacle) {
    if (!costmap || path.poses.empty()) {
        return false;
    }

    map::costmap_2d::Costmap2D* grid = costmap->getCostmap();
    if (!grid) {
        return false;
    }

    for (const auto& pose : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!grid->worldToMap(pose.pose.position.x, pose.pose.position.y, mx,
                              my)) {
            return false;
        }
        const unsigned char cost = grid->getCost(mx, my);
        if (cost == map::costmap_2d::NO_INFORMATION) {
            if (consider_unknown_as_obstacle) {
                return false;
            }
            continue;
        }
        if (cost >= max_cost) {
            return false;
        }
    }
    return true;
}

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy
