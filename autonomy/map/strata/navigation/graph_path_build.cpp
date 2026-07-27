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

#include <limits>

#include "autonomy/map/strata/navigation/graph_path_build.hpp"
#include "autonomy/map/strata/navigation/poi_util.hpp"
#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

namespace {

std::function<std::pair<double, double>(double, double)> MakeGpsToFracFn(const GeoUtils& geoUtils) {
    return [&geoUtils](double lng, double lat) {
        const LngLat ref0 = geoUtils.FracToGps(0., 0.);
        const LngLat ref1 = geoUtils.FracToGps(1., 1.);
        const double xFrac = (lng - ref0.x) / (ref1.x - ref0.x);
        const double yFrac = (lat - ref0.y) / (ref1.y - ref0.y);
        return std::pair<double, double>{xFrac, yFrac};
    };
}

std::vector<ObstaclePolygon> ToObstaclePolygons(const std::vector<FracPolygon>& polygons) {
    std::vector<ObstaclePolygon> result;
    result.reserve(polygons.size());
    for (size_t i = 0; i < polygons.size(); ++i) {
        ObstaclePolygon obstacle;
        obstacle.id = "building-" + std::to_string(i);
        for (const auto& point : polygons[i]) {
            LngLat vertex;
            vertex.x = point.first;
            vertex.y = point.second;
            obstacle.polygon.push_back(vertex);
        }
        result.push_back(std::move(obstacle));
    }
    return result;
}

bool SegmentCrossesBuildingPolygons(double x1, double y1, double x2, double y2,
                                    const std::vector<FracPolygon>& buildingPolygons) {
    for (const auto& polygon : buildingPolygons) {
        std::vector<LngLat> vertices;
        vertices.reserve(polygon.size());
        for (const auto& point : polygon) {
            vertices.push_back(LngLat{point.first, point.second, 0.});
        }
        if (utils::LineSegmentIntersectsPolygon(x1, y1, x2, y2, vertices)) {
            return true;
        }
    }
    return false;
}

}  // namespace

std::pair<double, double> ProjectToNearestGraphEdge(
    double xFrac, double yFrac, const RoadGraph& graph,
    const std::unordered_map<std::string, LngLat>& nodeIndex,
    const std::function<std::pair<double, double>(double, double)>& gpsToFracFn) {
    double minDistanceSq = std::numeric_limits<double>::infinity();
    std::pair<double, double> best{xFrac, yFrac};

    for (const auto& edge : graph.edges) {
        const auto fromIt = nodeIndex.find(edge.from);
        const auto toIt = nodeIndex.find(edge.to);
        if (fromIt == nodeIndex.end() || toIt == nodeIndex.end()) {
            continue;
        }
        const auto fromFrac = gpsToFracFn(fromIt->second.x, fromIt->second.y);
        const auto toFrac = gpsToFracFn(toIt->second.x, toIt->second.y);
        const double ax = fromFrac.first;
        const double ay = fromFrac.second;
        const double bx = toFrac.first;
        const double by = toFrac.second;
        const double dx = bx - ax;
        const double dy = by - ay;
        const double lengthSq = dx * dx + dy * dy;
        if (lengthSq == 0.) {
            continue;
        }
        double t = ((xFrac - ax) * dx + (yFrac - ay) * dy) / lengthSq;
        t = std::max(0., std::min(1., t));
        const double px = ax + t * dx;
        const double py = ay + t * dy;
        const double distSq = (px - xFrac) * (px - xFrac) + (py - yFrac) * (py - yFrac);
        if (distSq < minDistanceSq) {
            minDistanceSq = distSq;
            best = {px, py};
        }
    }
    return best;
}

BuiltRoute BuildGraphPointToPointPath(const RoadGraph& graph, GraphPathfinder& pathfinder,
                                      const std::string& startNodeId,
                                      const std::string& endNodeId,
                                      const std::function<LngLat(const std::string&)>& nodeToGps) {
    BuiltRoute result;
    const auto nodePath = pathfinder.FindPath(startNodeId, endNodeId);
    if (!nodePath) {
        return result;
    }
    for (const auto& nodeId : *nodePath) {
        const LngLat gps = nodeToGps(nodeId);
        result.coords.emplace_back(gps.x, gps.y);
    }
    if (!result.coords.empty()) {
        result.segmentIndices.push_back(result.coords.size() - 1);
    }
    return result;
}

std::vector<std::pair<double, double>> ValidateCollision(
    const std::vector<std::pair<double, double>>& coords,
    const std::function<bool(double, double, double, double)>& collisionCheck,
    const FindPathFn& repairFn) {
    if (coords.size() < 2) {
        return coords;
    }
    std::vector<std::pair<double, double>> result;
    result.push_back(coords.front());
    for (size_t i = 1; i < coords.size(); ++i) {
        const auto prev = result.back();
        const auto current = coords[i];
        if (collisionCheck(prev.first, prev.second, current.first, current.second)) {
            const auto repaired = repairFn(prev, current);
            if (repaired && repaired->size() > 1) {
                for (size_t j = 1; j < repaired->size(); ++j) {
                    result.push_back((*repaired)[j]);
                }
            } else {
                result.push_back(current);
            }
        } else {
            result.push_back(current);
        }
    }
    return result;
}

std::optional<std::pair<double, double>> ValidatePointCollision(
    double xFrac, double yFrac, const std::vector<FracPolygon>& buildingPolygons) {
    const auto obstacles = ToObstaclePolygons(buildingPolygons);
    if (!navigation::IsPointInForbiddenZone(xFrac, yFrac, obstacles)) {
        return std::nullopt;
    }
    return FindNearestBoundaryPoint(xFrac, yFrac, obstacles);
}

PathValidatorFn CreatePathValidator(PathValidatorConfig config) {
    return [config = std::move(config)](const std::vector<std::pair<double, double>>& coords) {
        if (coords.size() < 2) {
            return coords;
        }
        std::vector<std::pair<double, double>> result;
        result.push_back(coords.front());
        for (size_t i = 1; i < coords.size(); ++i) {
            const auto prev = result.back();
            const auto current = coords[i];
            bool segmentValid = true;
            if (config.collisionCheck) {
                segmentValid = !config.collisionCheck(prev.first, prev.second, current.first,
                                                      current.second);
            }
            bool precisionValid = true;
            if (config.deviationCheck) {
                precisionValid = config.deviationCheck(current, coords[i]);
            }
            if (segmentValid && precisionValid) {
                result.push_back(current);
                continue;
            }
            if (!segmentValid && config.onSegmentInvalid) {
                const auto repaired = config.onSegmentInvalid(prev, current, i);
                if (repaired && repaired->size() > 1) {
                    for (size_t j = 1; j < repaired->size(); ++j) {
                        result.push_back((*repaired)[j]);
                    }
                } else {
                    result.push_back(current);
                }
            } else {
                result.push_back(current);
            }
        }
        return result;
    };
}

RoadGraph RemoveEdgesThroughBuildings(const RoadGraph& graph, const GeoUtils& geoUtils,
                                      const std::vector<FracPolygon>& buildingPolygons,
                                      int sampleSteps) {
    if (buildingPolygons.empty()) {
        return graph;
    }
    const auto gpsToFrac = MakeGpsToFracFn(geoUtils);
    const auto nodeIndex = BuildNodeIndex(graph.nodes);
    RoadGraph filtered = graph;
    filtered.edges.clear();
    for (const auto& edge : graph.edges) {
        const auto fromIt = nodeIndex.find(edge.from);
        const auto toIt = nodeIndex.find(edge.to);
        if (fromIt == nodeIndex.end() || toIt == nodeIndex.end()) {
            continue;
        }
        const auto fromFrac = gpsToFrac(fromIt->second.x, fromIt->second.y);
        const auto toFrac = gpsToFrac(toIt->second.x, toIt->second.y);
        bool crosses = false;
        for (int step = 1; step < sampleSteps && !crosses; ++step) {
            const double t = static_cast<double>(step) / static_cast<double>(sampleSteps);
            const double px = fromFrac.first + (toFrac.first - fromFrac.first) * t;
            const double py = fromFrac.second + (toFrac.second - fromFrac.second) * t;
            for (const auto& polygon : buildingPolygons) {
                std::vector<LngLat> vertices;
                for (const auto& point : polygon) {
                    vertices.push_back(LngLat{point.first, point.second, 0.});
                }
                if (utils::PointInPolygon(px, py, vertices)) {
                    crosses = true;
                    break;
                }
            }
        }
        if (!crosses) {
            filtered.edges.push_back(edge);
        }
    }
    return filtered;
}

GraphCoveragePathResult BuildGraphCoveragePath(const std::vector<std::string>& nodeIdCircuit,
                                               const GraphCoveragePathOptions& options) {
    GraphCoveragePathResult result;
    if (nodeIdCircuit.empty()) {
        return result;
    }

    GraphPathfinder pathfinder(options.graph);
    const auto nodeIndex = BuildNodeIndex(options.graph.nodes);
    const auto gpsToFrac = MakeGpsToFracFn(options.geoUtils);
    const auto obstacles = ToObstaclePolygons(options.buildingPolygons);

    std::vector<std::pair<double, double>> rawCoords;
    rawCoords.reserve(nodeIdCircuit.size());
    for (const auto& nodeId : nodeIdCircuit) {
        const auto it = nodeIndex.find(nodeId);
        if (it == nodeIndex.end()) {
            continue;
        }
        auto frac = gpsToFrac(it->second.x, it->second.y);
        if (navigation::IsPointInForbiddenZone(frac.first, frac.second, obstacles)) {
            const auto corrected = FindNearestBoundaryPoint(frac.first, frac.second, obstacles);
            frac = corrected;
            const auto projected = ProjectToNearestGraphEdge(
                frac.first, frac.second, options.graph, nodeIndex, gpsToFrac);
            frac = projected;
        }
        const auto cart = options.geoUtils.FracToCart(frac.first, frac.second);
        rawCoords.emplace_back(cart.x, cart.y);
    }

    PathValidatorConfig validatorConfig;
    validatorConfig.collisionCheck =
        [&options, &gpsToFrac](double x1, double y1, double x2, double y2) {
            const LngLat gps1 = options.geoUtils.CartToGps(x1, y1);
            const LngLat gps2 = options.geoUtils.CartToGps(x2, y2);
            const auto frac1 = gpsToFrac(gps1.x, gps1.y);
            const auto frac2 = gpsToFrac(gps2.x, gps2.y);
            return SegmentCrossesBuildingPolygons(frac1.first, frac1.second, frac2.first,
                                                frac2.second, options.buildingPolygons);
        };
    validatorConfig.onSegmentInvalid =
        [&options, &pathfinder, &gpsToFrac, &result](const std::pair<double, double>& prev,
                                                     const std::pair<double, double>& current,
                                                     size_t segmentIndex) {
            const LngLat startGps = options.geoUtils.CartToGps(prev.first, prev.second);
            const LngLat endGps = options.geoUtils.CartToGps(current.first, current.second);
            const auto startNode = pathfinder.FindNearestNode(startGps);
            const auto endNode = pathfinder.FindNearestNode(endGps);
            if (!startNode || !endNode || *startNode == *endNode) {
                return std::optional<std::vector<std::pair<double, double>>>{};
            }
            const auto altPath = pathfinder.FindPath(*startNode, *endNode, options.excludeNodeIds);
            if (!altPath || altPath->size() <= 1) {
                return std::optional<std::vector<std::pair<double, double>>>{};
            }
            std::vector<std::pair<double, double>> repaired;
            repaired.push_back(prev);
            const auto nodeIndex = BuildNodeIndex(options.graph.nodes);
            for (size_t i = 1; i < altPath->size(); ++i) {
                const auto nodeIt = nodeIndex.find((*altPath)[i]);
                if (nodeIt == nodeIndex.end()) {
                    continue;
                }
                auto frac = gpsToFrac(nodeIt->second.x, nodeIt->second.y);
                const auto obstacles = ToObstaclePolygons(options.buildingPolygons);
                if (navigation::IsPointInForbiddenZone(frac.first, frac.second, obstacles)) {
                    frac = FindNearestBoundaryPoint(frac.first, frac.second, obstacles);
                    frac = ProjectToNearestGraphEdge(frac.first, frac.second, options.graph,
                                                     nodeIndex, gpsToFrac);
                }
                const auto cart = options.geoUtils.FracToCart(frac.first, frac.second);
                repaired.emplace_back(cart.x, cart.y);
            }
            result.reRouteCount++;
            if (options.onRerouteLog) {
                options.onRerouteLog(*startNode, *endNode, segmentIndex);
            }
            return std::optional<std::vector<std::pair<double, double>>>{std::move(repaired)};
        };

    result.coords = CreatePathValidator(std::move(validatorConfig))(rawCoords);
    return result;
}

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
