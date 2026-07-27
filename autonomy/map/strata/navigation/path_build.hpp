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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/navigation/pathfinder.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

using FindPathFn = std::function<std::optional<std::vector<std::pair<double, double>>>(
    const std::pair<double, double>&, const std::pair<double, double>&)>;

BuiltRoute BuildRoute(const std::vector<RouteSegment>& segments, const FindPathFn& findPathFn);

class PathBuildService
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(PathBuildService)

    void InitPathfinder(double widthMeters, double heightMeters,
                        const PathfinderOptions& options = {});

    PathfindingRouteResult BuildPathfindingRoute(
        const std::vector<std::pair<double, double>>& points,
        const std::vector<ObstaclePolygon>& forbiddenZones);

    std::vector<std::pair<double, double>> ValidatePathSegments(
        const std::vector<std::pair<double, double>>& coords,
        const std::vector<ObstaclePolygon>& forbiddenZones);

private:
    std::pair<double, double> AdjustStartPoint(
        const std::pair<double, double>& point,
        const std::vector<ObstaclePolygon>& forbiddenZones) const;
    std::pair<double, double> AdjustEndPoint(
        const std::pair<double, double>& point,
        const std::pair<double, double>& referencePoint,
        const std::vector<ObstaclePolygon>& forbiddenZones) const;
    void EnsureObstaclesSet(const std::vector<ObstaclePolygon>& forbiddenZones);

    Pathfinder::SharedPtr pathfinder_;
    std::string lastObstacleKey_;
};

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
