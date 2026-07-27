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
#include <stdexcept>

#include "autonomy/map/strata/navigation/path_build.hpp"
#include "autonomy/map/strata/navigation/poi_util.hpp"
#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

BuiltRoute BuildRoute(const std::vector<RouteSegment>& segments, const FindPathFn& findPathFn) {
    BuiltRoute result;
    if (segments.empty()) {
        return result;
    }

    std::pair<double, double> lastCoord{};
    bool hasLast = false;
    for (const auto& segment : segments) {
        const auto segmentPath = findPathFn(segment.start, segment.end);
        if (segmentPath && segmentPath->size() >= 2) {
            size_t startOffset = 0;
            if (hasLast && (*segmentPath)[0] == lastCoord) {
                startOffset = 1;
            }
            for (size_t i = startOffset; i < segmentPath->size(); ++i) {
                result.coords.push_back((*segmentPath)[i]);
            }
        } else {
            result.coords.push_back(segment.end);
        }
        result.segmentIndices.push_back(result.coords.size() - 1);
        lastCoord = result.coords.back();
        hasLast = true;
    }
    return result;
}

void PathBuildService::InitPathfinder(double widthMeters, double heightMeters,
                                      const PathfinderOptions& options) {
    PathfinderOptions opts = options;
    opts.widthMeters = widthMeters;
    opts.heightMeters = heightMeters;
    pathfinder_ = Pathfinder::make_shared(opts);
    lastObstacleKey_.clear();
}

void PathBuildService::EnsureObstaclesSet(const std::vector<ObstaclePolygon>& forbiddenZones) {
    if (!pathfinder_) {
        throw std::runtime_error("Pathfinder not initialized, call InitPathfinder first");
    }
    std::vector<std::string> ids;
    ids.reserve(forbiddenZones.size());
    for (const auto& zone : forbiddenZones) {
        ids.push_back(zone.id);
    }
    std::sort(ids.begin(), ids.end());
    std::string key;
    for (const auto& id : ids) {
        key += id + ",";
    }
    if (key.empty()) {
        key = "default";
    }
    if (key != lastObstacleKey_) {
        pathfinder_->SetObstacles(forbiddenZones);
        lastObstacleKey_ = key;
    }
}

std::pair<double, double> PathBuildService::AdjustStartPoint(
    const std::pair<double, double>& point,
    const std::vector<ObstaclePolygon>& forbiddenZones) const {
    if (IsPointInForbiddenZone(point.first, point.second, forbiddenZones)) {
        return FindNearestBoundaryPoint(point.first, point.second, forbiddenZones);
    }
    return point;
}

std::pair<double, double> PathBuildService::AdjustEndPoint(
    const std::pair<double, double>& point, const std::pair<double, double>& referencePoint,
    const std::vector<ObstaclePolygon>& forbiddenZones) const {
    if (!IsPointInForbiddenZone(point.first, point.second, forbiddenZones)) {
        return point;
    }
    AnnouncementPointOptions opts;
    opts.poiXFrac = point.first;
    opts.poiYFrac = point.second;
    opts.prevXFrac = referencePoint.first;
    opts.prevYFrac = referencePoint.second;
    opts.forbiddenZones = &forbiddenZones;
    return FindAnnouncementPoint(opts);
}

PathfindingRouteResult PathBuildService::BuildPathfindingRoute(
    const std::vector<std::pair<double, double>>& points,
    const std::vector<ObstaclePolygon>& forbiddenZones) {
    PathfindingRouteResult result;
    if (points.size() < 2) {
        return result;
    }
    EnsureObstaclesSet(forbiddenZones);

    std::vector<RouteSegment> segments;
    segments.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& current = points[i];
        const auto& next = points[(i + 1) % points.size()];
        segments.push_back({AdjustStartPoint(current, forbiddenZones),
                          AdjustEndPoint(next, current, forbiddenZones)});
    }

    const auto built =
        BuildRoute(segments, [this](const std::pair<double, double>& start,
                                    const std::pair<double, double>& end) {
            return pathfinder_->FindPath(start, end);
        });

    if (!built.segmentIndices.empty()) {
        result.poiIndices.assign(built.segmentIndices.begin(), built.segmentIndices.end() - 1);
    }
    result.coords = ValidatePathSegments(built.coords, forbiddenZones);
    return result;
}

std::vector<std::pair<double, double>> PathBuildService::ValidatePathSegments(
    const std::vector<std::pair<double, double>>& coords,
    const std::vector<ObstaclePolygon>& forbiddenZones) {
    if (coords.size() < 2 || forbiddenZones.empty() || !pathfinder_) {
        return coords;
    }

    std::vector<std::pair<double, double>> result;
    result.push_back(coords.front());
    for (size_t i = 1; i < coords.size(); ++i) {
        const auto prev = result.back();
        const auto current = coords[i];
        bool intersects = false;
        for (const auto& zone : forbiddenZones) {
            if (utils::LineSegmentIntersectsPolygon(prev.first, prev.second, current.first,
                                                    current.second, zone.polygon)) {
                intersects = true;
                break;
            }
        }
        if (intersects) {
            const double midX = (prev.first + current.first) / 2.;
            const double midY = (prev.second + current.second) / 2.;
            const auto midPath = pathfinder_->FindPath(prev, {midX, midY});
            if (midPath && midPath->size() >= 2) {
                for (size_t j = 1; j < midPath->size(); ++j) {
                    result.push_back((*midPath)[j]);
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

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
