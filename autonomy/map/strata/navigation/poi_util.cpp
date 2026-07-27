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

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

#include "autonomy/map/strata/navigation/poi_util.hpp"
#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

namespace {

std::pair<double, double> ClosestPointOnSegment(double px, double py, double ax, double ay,
                                                double bx, double by) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double lenSq = dx * dx + dy * dy;
    if (lenSq == 0.) {
        return {ax, ay};
    }
    double t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
    t = std::max(0., std::min(1., t));
    return {ax + t * dx, ay + t * dy};
}

const ObstaclePolygon* FindContainingZone(double xFrac, double yFrac,
                                          const std::vector<ObstaclePolygon>& zones) {
    for (const auto& zone : zones) {
        if (utils::PointInPolygon(xFrac, yFrac, zone.polygon)) {
            return &zone;
        }
    }
    return nullptr;
}

}  // namespace

bool IsPointInForbiddenZone(double xFrac, double yFrac,
                            const std::vector<ObstaclePolygon>& forbiddenZones) {
    return FindContainingZone(xFrac, yFrac, forbiddenZones) != nullptr;
}

std::pair<double, double> FindNearestBoundaryPoint(
    double xFrac, double yFrac, const std::vector<ObstaclePolygon>& forbiddenZones,
    double pushDistance) {
    const auto* zone = FindContainingZone(xFrac, yFrac, forbiddenZones);
    if (!zone) {
        return {xFrac, yFrac};
    }

    double minDistSq = std::numeric_limits<double>::infinity();
    std::pair<double, double> nearest{xFrac, yFrac};
    const auto& polygon = zone->polygon;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto closest = ClosestPointOnSegment(xFrac, yFrac, polygon[j].x, polygon[j].y,
                                                   polygon[i].x, polygon[i].y);
        const double dx = xFrac - closest.first;
        const double dy = yFrac - closest.second;
        const double distSq = dx * dx + dy * dy;
        if (distSq < minDistSq) {
            minDistSq = distSq;
            nearest = closest;
        }
    }

    const double nx = nearest.first;
    const double ny = nearest.second;
    double ex = xFrac - nx;
    double ey = yFrac - ny;
    const double len = std::hypot(ex, ey);
    if (len > 0.) {
        ex /= len;
        ey /= len;
        return {nx - ex * pushDistance, ny - ey * pushDistance};
    }

    double cxSum = 0.;
    double cySum = 0.;
    for (const auto& pt : polygon) {
        cxSum += pt.x;
        cySum += pt.y;
    }
    const double centerX = cxSum / polygon.size();
    const double centerY = cySum / polygon.size();
    const double dirX = xFrac - centerX;
    const double dirY = yFrac - centerY;
    const double dirLen = std::hypot(dirX, dirY);
    if (dirLen > 0.) {
        return {nx + dirX / dirLen * pushDistance, ny + dirY / dirLen * pushDistance};
    }
    return {nx + pushDistance, ny};
}

std::pair<double, double> FindForbiddenZoneCornerTarget(
    double poiXFrac, double poiYFrac, double robotXFrac, double robotYFrac,
    const std::vector<ObstaclePolygon>& forbiddenZones, double pushDistance) {
    const auto* zone = FindContainingZone(poiXFrac, poiYFrac, forbiddenZones);
    if (!zone) {
        return {poiXFrac, poiYFrac};
    }

    const auto& polygon = zone->polygon;
    double minDistSq = std::numeric_limits<double>::infinity();
    std::pair<double, double> nearestCorner{poiXFrac, poiYFrac};
    for (const auto& vertex : polygon) {
        const double dx = vertex.x - robotXFrac;
        const double dy = vertex.y - robotYFrac;
        const double distSq = dx * dx + dy * dy;
        if (distSq < minDistSq) {
            minDistSq = distSq;
            nearestCorner = {vertex.x, vertex.y};
        }
    }

    double cxSum = 0.;
    double cySum = 0.;
    for (const auto& pt : polygon) {
        cxSum += pt.x;
        cySum += pt.y;
    }
    const double centerX = cxSum / polygon.size();
    const double centerY = cySum / polygon.size();
    const double dirX = centerX - nearestCorner.first;
    const double dirY = centerY - nearestCorner.second;
    const double dirLen = std::hypot(dirX, dirY);
    if (dirLen <= 0.) {
        return nearestCorner;
    }
    return {nearestCorner.first + dirX / dirLen * pushDistance,
            nearestCorner.second + dirY / dirLen * pushDistance};
}

std::pair<double, double> FindAnnouncementPoint(const AnnouncementPointOptions& options) {
    if (!options.forbiddenZones) {
        return {options.poiXFrac, options.poiYFrac};
    }
    const auto* zone =
        FindContainingZone(options.poiXFrac, options.poiYFrac, *options.forbiddenZones);
    if (!zone) {
        return {options.poiXFrac, options.poiYFrac};
    }

    std::vector<LngLat> uniqueVerts;
    std::set<std::string> seen;
    for (const auto& vertex : zone->polygon) {
        const std::string key =
            std::to_string(vertex.x) + "," + std::to_string(vertex.y);
        if (seen.insert(key).second) {
            uniqueVerts.push_back(vertex);
        }
    }

    struct VertexDist {
        LngLat vertex;
        double distanceSq{0.};
    };
    std::vector<VertexDist> distances;
    distances.reserve(uniqueVerts.size());
    for (const auto& vertex : uniqueVerts) {
        const double dx = vertex.x - options.prevXFrac;
        const double dy = vertex.y - options.prevYFrac;
        distances.push_back({vertex, dx * dx + dy * dy});
    }
    std::sort(distances.begin(), distances.end(),
              [](const VertexDist& a, const VertexDist& b) { return a.distanceSq < b.distanceSq; });
    if (distances.size() < 2) {
        return {options.poiXFrac, options.poiYFrac};
    }

    const double midX = (distances[0].vertex.x + distances[1].vertex.x) / 2.;
    const double midY = (distances[0].vertex.y + distances[1].vertex.y) / 2.;
    double centerX = 0.;
    double centerY = 0.;
    for (const auto& vertex : uniqueVerts) {
        centerX += vertex.x;
        centerY += vertex.y;
    }
    centerX /= uniqueVerts.size();
    centerY /= uniqueVerts.size();

    const double directionX = midX - centerX;
    const double directionY = midY - centerY;
    const double directionLength = std::hypot(directionX, directionY);
    if (directionLength == 0.) {
        return {midX, midY};
    }
    return {midX + directionX / directionLength * options.meterToFrac,
            midY + directionY / directionLength * options.meterToFrac};
}

std::vector<std::pair<double, double>> RouteIdsToCoords(
    const std::vector<std::string>& routeIds, const std::vector<PoiFracPoint>& allPois) {
    std::unordered_map<std::string, PoiFracPoint> poiMap;
    for (const auto& poi : allPois) {
        poiMap[poi.id] = poi;
    }
    std::vector<std::pair<double, double>> coords;
    coords.reserve(routeIds.size());
    for (const auto& routeId : routeIds) {
        const auto it = poiMap.find(routeId);
        if (it == poiMap.end()) {
            coords.emplace_back(0.5, 0.5);
        } else {
            coords.emplace_back(it->second.xFrac, it->second.yFrac);
        }
    }
    return coords;
}

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
