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

#include <string>
#include <utility>
#include <vector>

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

bool IsPointInForbiddenZone(double xFrac, double yFrac,
                            const std::vector<ObstaclePolygon>& forbiddenZones);

std::pair<double, double> FindNearestBoundaryPoint(
    double xFrac, double yFrac, const std::vector<ObstaclePolygon>& forbiddenZones,
    double pushDistance = 0.01);

std::pair<double, double> FindForbiddenZoneCornerTarget(
    double poiXFrac, double poiYFrac, double robotXFrac, double robotYFrac,
    const std::vector<ObstaclePolygon>& forbiddenZones, double pushDistance = 0.01);

struct AnnouncementPointOptions {
    double poiXFrac{0.};
    double poiYFrac{0.};
    double prevXFrac{0.};
    double prevYFrac{0.};
    const std::vector<ObstaclePolygon>* forbiddenZones{nullptr};
    double meterToFrac{0.01};
};

std::pair<double, double> FindAnnouncementPoint(const AnnouncementPointOptions& options);

struct PoiFracPoint {
    std::string id;
    double xFrac{0.};
    double yFrac{0.};
};

std::vector<std::pair<double, double>> RouteIdsToCoords(
    const std::vector<std::string>& routeIds, const std::vector<PoiFracPoint>& allPois);

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
