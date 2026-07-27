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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/strata/types.hpp"
#include "autonomy/map/strata/utils/map_utils.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

struct GeoUtilsConfig {
    double startX{0.};
    double startY{0.};
    double width{0.};
    double height{0.};
    double scale{0.01};
    double zoomFactor{2.0};
    double originLatitude{utils::kDefaultOriginLatitude};
    double originLongitude{utils::kDefaultOriginLongitude};
};

class GeoUtils
{
public:
    explicit GeoUtils(const GeoUtilsConfig& config);

    commsgs::geometry_msgs::Point FracToCart(double xFrac, double yFrac) const;
    LngLat CartToGps(double x, double y) const;
    LngLat FracToGps(double xFrac, double yFrac) const;

    static double CartDist(const commsgs::geometry_msgs::Point& a,
                         const commsgs::geometry_msgs::Point& b);
    static double CartHeading(const commsgs::geometry_msgs::Point& from,
                              const commsgs::geometry_msgs::Point& to);
    static double IconRot(double headingDeg);
    static double LerpAngle(double fromDeg, double toDeg, double t);

private:
    GeoUtilsConfig config_;
};

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
