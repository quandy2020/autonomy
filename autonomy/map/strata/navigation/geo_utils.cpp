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

#include "autonomy/map/strata/navigation/geo_utils.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

GeoUtils::GeoUtils(const GeoUtilsConfig& config) : config_(config) {}

commsgs::geometry_msgs::Point GeoUtils::FracToCart(double xFrac, double yFrac) const {
    commsgs::geometry_msgs::Point p;
    p.x = config_.startX + xFrac * config_.width;
    p.y = config_.startY + yFrac * config_.height;
    return p;
}

LngLat GeoUtils::CartToGps(double x, double y) const {
    utils::CartesianToGpsParams params;
    params.x = x;
    params.y = y;
    params.scale = config_.scale;
    params.zoomFactor = config_.zoomFactor;
    params.originLatitude = config_.originLatitude;
    params.originLongitude = config_.originLongitude;
    const auto gps = utils::CartesianToGps(params);
    LngLat ll;
    ll.x = gps.longitude;
    ll.y = gps.latitude;
    return ll;
}

LngLat GeoUtils::FracToGps(double xFrac, double yFrac) const {
    const auto cart = FracToCart(xFrac, yFrac);
    return CartToGps(cart.x, cart.y);
}

double GeoUtils::CartDist(const commsgs::geometry_msgs::Point& a,
                          const commsgs::geometry_msgs::Point& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double GeoUtils::CartHeading(const commsgs::geometry_msgs::Point& from,
                             const commsgs::geometry_msgs::Point& to) {
    return std::fmod(std::atan2(to.x - from.x, to.y - from.y) * 180.0 / M_PI + 360.0, 360.0);
}

double GeoUtils::IconRot(double headingDeg) {
    return std::fmod(headingDeg - 90.0 + 360.0, 360.0);
}

double GeoUtils::LerpAngle(double fromDeg, double toDeg, double t) {
    const double diff = std::fmod(toDeg - fromDeg + 540.0, 360.0) - 180.0;
    return std::fmod(fromDeg + diff * t + 360.0, 360.0);
}

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
