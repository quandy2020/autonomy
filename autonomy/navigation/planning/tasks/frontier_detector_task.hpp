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

/**
 * @file frontier_detector_task.hpp
 * @brief Finds free/unknown boundaries and returns exploration waypoints.
 */

#pragma once

#include <vector>

#include "navigation/common/pose2d.hpp"
#include "automsgs/msgs/map_msgs/occupancy_grid.pb.h"
#include "navigation/planning/base/waypoint.hpp"

namespace navigation
{
namespace planning
{

class FrontierDetectorTask
{
public:
    /**
     * @brief Return reachable frontier clusters ordered by distance.
     *
     * @param[in] grid automsgs occupancy grid; -1 unknown, 0 free, 100 occupied.
     * @param[in] robot_pose Current pose.
     * @param[in] max_range Maximum waypoint distance (<=0 disables).
     */
    [[nodiscard]] std::vector<Waypoint> Detect(
        const automsgs::msgs::map_msgs::OccupancyGrid& grid,
        const common::Pose2D& robot_pose, double max_range) const;
};

}  // namespace planning
}  // namespace navigation
