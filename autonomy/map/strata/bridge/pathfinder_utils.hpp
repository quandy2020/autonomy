/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#pragma once

#include <utility>
#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace bridge {

inline std::pair<double, double> MapSizeFromSlam(const SlamMapOptions& options) {
    if (options.xGridCount <= 0 || options.yGridCount <= 0 || options.resolution <= 0.) {
        return {0., 0.};
    }
    return {options.xGridCount * options.resolution, options.yGridCount * options.resolution};
}

/** 禁行语义区 → Pathfinder 分数坐标障碍（BICMap securityPatrol 同型）。 */
inline std::vector<ObstaclePolygon> BuildPathfinderObstacles(
    const std::vector<render::SemanticZoneFeature>& zones, double width_meters,
    double height_meters) {
    std::vector<ObstaclePolygon> obstacles;
    if (width_meters <= 0. || height_meters <= 0.) {
        return obstacles;
    }
    for (const auto& zone : zones) {
        if (zone.type != ZoneType::kForbidden || zone.polygon.size() < 3) {
            continue;
        }
        ObstaclePolygon obstacle;
        obstacle.id = zone.id;
        obstacle.polygon.reserve(zone.polygon.size());
        for (const auto& point : zone.polygon) {
            LngLat frac;
            frac.x = point.x / width_meters;
            frac.y = point.y / height_meters;
            frac.z = point.z;
            obstacle.polygon.push_back(frac);
        }
        obstacles.push_back(std::move(obstacle));
    }
    return obstacles;
}

inline commsgs::planning_msgs::Path BuildPathMessage(
    const std::vector<LngLat>& waypoints, const commsgs::std_msgs::Header& header) {
    commsgs::planning_msgs::Path path;
    path.header = header;
    path.poses.reserve(waypoints.size());
    for (const auto& point : waypoints) {
        commsgs::geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.;
        path.poses.push_back(std::move(pose));
    }
    return path;
}

}  // namespace bridge
}  // namespace strata
}  // namespace map
}  // namespace autonomy
