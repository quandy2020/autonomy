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

#include "autonomy/map/strata/shapes/shape_controller.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace shapes {

std::vector<LngLat> PathToWidePolygon(const std::vector<LngLat>& path, double widthM) {
    if (path.size() < 2) {
        return {};
    }
    const double half = widthM / 2.0;
    std::vector<LngLat> left;
    std::vector<LngLat> right;

    for (size_t i = 0; i < path.size(); ++i) {
        double dx = 0.;
        double dy = 0.;
        if (i + 1 < path.size()) {
            dx = path[i + 1].x - path[i].x;
            dy = path[i + 1].y - path[i].y;
        } else if (i > 0) {
            dx = path[i].x - path[i - 1].x;
            dy = path[i].y - path[i - 1].y;
        }
        const double len = std::hypot(dx, dy);
        if (len < 1e-10) {
            continue;
        }
        const double nx = -dy / len * half;
        const double ny = dx / len * half;
        LngLat l;
        l.x = path[i].x + nx;
        l.y = path[i].y + ny;
        LngLat r;
        r.x = path[i].x - nx;
        r.y = path[i].y - ny;
        left.push_back(l);
        right.push_back(r);
    }

    std::vector<LngLat> polygon = left;
    polygon.insert(polygon.end(), right.rbegin(), right.rend());
    if (!polygon.empty()) {
        polygon.push_back(polygon.front());
    }
    return polygon;
}

}  // namespace shapes
}  // namespace strata
}  // namespace map
}  // namespace autonomy
