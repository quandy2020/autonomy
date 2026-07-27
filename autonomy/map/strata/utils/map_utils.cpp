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

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/utils/map_utils.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

double GetDistance(double lat1, double lon1, double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * M_PI / 180.0;
    const double dLon = (lon2 - lon1) * M_PI / 180.0;
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                     std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
                         std::sin(dLon / 2) * std::sin(dLon / 2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return kEarthRadiusM * c;
}

void ValidateDistance(double originLat, double originLon, double targetLat,
                      double targetLon) {
    const double distance = GetDistance(originLat, originLon, targetLat, targetLon);
    if (distance > kMaxDistanceM) {
        throw std::runtime_error("Distance " + std::to_string(distance) +
                                 "m exceeds max allowed " + std::to_string(kMaxDistanceM) +
                                 "m");
    }
}

MapCorners GetMapCorners(double startX, double startY, int xGridCount, int yGridCount,
                         double resolution) {
    MapCorners corners;
    corners.bottomLeft.x = startX;
    corners.bottomLeft.y = startY;
    corners.bottomRight.x = startX + xGridCount * resolution;
    corners.bottomRight.y = startY;
    corners.topLeft.x = startX;
    corners.topLeft.y = startY + yGridCount * resolution;
    corners.topRight.x = startX + xGridCount * resolution;
    corners.topRight.y = startY + yGridCount * resolution;
    return corners;
}

GpsPoint CartesianToGps(const CartesianToGpsParams& params) {
    const double adjustedScale = params.scale * params.zoomFactor;
    const double distanceX = params.x * adjustedScale;
    const double distanceY = params.y * adjustedScale;

    const double deltaLat =
        (distanceY * std::cos(params.bearing) - distanceX * std::sin(params.bearing)) /
        kEarthRadiusM;
    const double deltaLon =
        (distanceX * std::cos(params.bearing) + distanceY * std::sin(params.bearing)) /
        (kEarthRadiusM * std::cos(params.originLatitude * M_PI / 180.0));

    GpsPoint result;
    result.latitude = params.originLatitude + deltaLat * 180.0 / M_PI;
    result.longitude = params.originLongitude + deltaLon * 180.0 / M_PI;

    const double distance = GetDistance(params.originLatitude, params.originLongitude,
                                        result.latitude, result.longitude);
    if (distance > kMaxDistanceM) {
        AWARN << "Distance " << distance << "m exceeds recommended max "
              << kMaxDistanceM << "m";
    }
    return result;
}

CartesianPoint GpsToCartesian(const GpsToCartesianParams& params) {
    const double distance = GetDistance(params.originLatitude, params.originLongitude,
                                        params.latitude, params.longitude);
    if (distance > kMaxDistanceM) {
        AWARN << "Distance " << distance << "m exceeds recommended max "
              << kMaxDistanceM << "m";
    }

    const double deltaLat = (params.latitude - params.originLatitude) * M_PI / 180.0;
    const double deltaLon = (params.longitude - params.originLongitude) * M_PI / 180.0;

    const double y = deltaLat * kEarthRadiusM;
    const double x = deltaLon * kEarthRadiusM * std::cos(params.originLatitude * M_PI / 180.0);

    const double adjustedScale = params.scale * params.zoomFactor;
    CartesianPoint result;
    result.x = (x * std::cos(-params.bearing) - y * std::sin(-params.bearing)) / adjustedScale;
    result.y = (x * std::sin(-params.bearing) + y * std::cos(-params.bearing)) / adjustedScale;
    return result;
}

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
