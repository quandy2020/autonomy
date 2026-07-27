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

#include "autonomy/map/strata/robot/visual/fov_geometry.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace visual {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

std::vector<RobotFovBand> BuildFovBands(const LngLat& position, double heading_deg,
                                        const RobotFovOptions& options) {
    std::vector<RobotFovBand> bands;
    if (!options.enabled) {
        return bands;
    }
    const int band_count = std::max(2, options.bands);
    const int segments = std::max(2, options.segments);
    const double radius = options.radiusMeters;
    const double half_angle = options.angleDeg / 2.0;
    const double heading = std::fmod(heading_deg + 360.0, 360.0);

    for (int band = 0; band < band_count; ++band) {
        const double r_inner = radius * (static_cast<double>(band) / band_count);
        const double r_outer = radius * (static_cast<double>(band + 1) / band_count);
        const double t = (static_cast<double>(band) + 0.5) / band_count;
        const float opacity = options.innerOpacity +
                              (options.outerOpacity - options.innerOpacity) *
                                  static_cast<float>(t);

        RobotFovBand fov_band;
        fov_band.opacity = opacity;

        for (int i = 0; i <= segments; ++i) {
            const double angle = -half_angle + (options.angleDeg * i / segments);
            const double rad = (heading + angle) * kPi / 180.0;
            LngLat point;
            point.x = position.x + r_outer * std::sin(rad);
            point.y = position.y + r_outer * std::cos(rad);
            fov_band.polygon.push_back(point);
        }
        for (int i = segments; i >= 0; --i) {
            const double angle = -half_angle + (options.angleDeg * i / segments);
            const double rad = (heading + angle) * kPi / 180.0;
            LngLat point;
            point.x = position.x + r_inner * std::sin(rad);
            point.y = position.y + r_inner * std::cos(rad);
            fov_band.polygon.push_back(point);
        }
        if (!fov_band.polygon.empty()) {
            fov_band.polygon.push_back(fov_band.polygon.front());
        }
        bands.push_back(std::move(fov_band));
    }
    return bands;
}

}  // namespace visual
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
