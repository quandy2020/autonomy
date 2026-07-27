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

#include <cmath>
#include <stdexcept>

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kMaxDistanceM = 5000.0;
constexpr double kDefaultOriginLatitude = 39.9042;
constexpr double kDefaultOriginLongitude = 116.4074;
constexpr double kDefaultScale = 0.01;

double GetDistance(double lat1, double lon1, double lat2, double lon2);
void ValidateDistance(double originLat, double originLon, double targetLat, double targetLon);

MapCorners GetMapCorners(double startX, double startY, int xGridCount, int yGridCount,
                         double resolution);

struct CartesianToGpsParams {
    double x{0.};
    double y{0.};
    double originLatitude{kDefaultOriginLatitude};
    double originLongitude{kDefaultOriginLongitude};
    double scale{kDefaultScale};
    double bearing{0.};
    double zoomFactor{1.0};
};

struct GpsPoint {
    double latitude{0.};
    double longitude{0.};
};

GpsPoint CartesianToGps(const CartesianToGpsParams& params);

struct GpsToCartesianParams {
    double latitude{0.};
    double longitude{0.};
    double originLatitude{kDefaultOriginLatitude};
    double originLongitude{kDefaultOriginLongitude};
    double scale{kDefaultScale};
    double bearing{0.};
    double zoomFactor{1.0};
};

struct CartesianPoint {
    double x{0.};
    double y{0.};
};

CartesianPoint GpsToCartesian(const GpsToCartesianParams& params);

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
