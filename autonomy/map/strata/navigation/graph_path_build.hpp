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
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autonomy/map/strata/navigation/geo_utils.hpp"
#include "autonomy/map/strata/navigation/graph_pathfinder.hpp"
#include "autonomy/map/strata/navigation/path_build.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

constexpr int kDefaultBuildingSampleSteps = 24;

std::pair<double, double> ProjectToNearestGraphEdge(
    double xFrac, double yFrac, const RoadGraph& graph,
    const std::unordered_map<std::string, LngLat>& nodeIndex,
    const std::function<std::pair<double, double>(double, double)>& gpsToFracFn);

BuiltRoute BuildGraphPointToPointPath(const RoadGraph& graph, GraphPathfinder& pathfinder,
                                      const std::string& startNodeId,
                                      const std::string& endNodeId,
                                      const std::function<LngLat(const std::string&)>& nodeToGps);

std::vector<std::pair<double, double>> ValidateCollision(
    const std::vector<std::pair<double, double>>& coords,
    const std::function<bool(double, double, double, double)>& collisionCheck,
    const FindPathFn& repairFn);

/** BICMap validateCollision(xFrac, yFrac, buildingPolygons)。 */
std::optional<std::pair<double, double>> ValidatePointCollision(
    double xFrac, double yFrac, const std::vector<FracPolygon>& buildingPolygons);

struct PathValidatorConfig {
    std::function<bool(double, double, double, double)> collisionCheck;
    std::function<std::optional<std::vector<std::pair<double, double>>>(
        const std::pair<double, double>&, const std::pair<double, double>&, size_t)>
        onSegmentInvalid;
    std::function<bool(const std::pair<double, double>&, const std::pair<double, double>&)>
        deviationCheck;
};

using PathValidatorFn = std::function<std::vector<std::pair<double, double>>(
    const std::vector<std::pair<double, double>>&)>;

/** BICMap createPathValidator(config)。 */
PathValidatorFn CreatePathValidator(PathValidatorConfig config);

/** BICMap removeEdgesThroughBuildings(graph, options)。 */
RoadGraph RemoveEdgesThroughBuildings(
    const RoadGraph& graph, const GeoUtils& geoUtils,
    const std::vector<FracPolygon>& buildingPolygons, int sampleSteps = kDefaultBuildingSampleSteps);

struct GraphCoveragePathResult {
    std::vector<std::pair<double, double>> coords;
    size_t reRouteCount{0};
};

struct GraphCoveragePathOptions {
    GeoUtils geoUtils;
    RoadGraph graph;
    std::vector<FracPolygon> buildingPolygons;
    std::unordered_set<std::string> excludeNodeIds;
    std::function<void(const std::string&, const std::string&, size_t)> onRerouteLog;
};

/** BICMap buildGraphCoveragePath(nodeIdCircuit, options)。 */
GraphCoveragePathResult BuildGraphCoveragePath(const std::vector<std::string>& nodeIdCircuit,
                                               const GraphCoveragePathOptions& options);

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
