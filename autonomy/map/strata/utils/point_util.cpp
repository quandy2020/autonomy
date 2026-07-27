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

#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

bool PointInPolygon(double px, double py, const std::vector<LngLat>& polygon) {
    bool inside = false;
    const int n = static_cast<int>(polygon.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].x;
        const double yi = polygon[i].y;
        const double xj = polygon[j].x;
        const double yj = polygon[j].y;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi + 1e-20) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

bool SegmentsIntersect(double ax, double ay, double bx, double by, double cx, double cy,
                       double dx, double dy) {
    const double d1x = bx - ax;
    const double d1y = by - ay;
    const double d2x = dx - cx;
    const double d2y = dy - cy;
    const double cross = d1x * d2y - d1y * d2x;
    if (std::abs(cross) < 1e-10) {
        return false;
    }
    const double t = ((cx - ax) * d2y - (cy - ay) * d2x) / cross;
    const double u = ((cx - ax) * d1y - (cy - ay) * d1x) / cross;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

bool LineSegmentIntersectsPolygon(double x1, double y1, double x2, double y2,
                                  const std::vector<LngLat>& polygon) {
    const int n = static_cast<int>(polygon.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (SegmentsIntersect(x1, y1, x2, y2, polygon[i].x, polygon[i].y, polygon[j].x,
                              polygon[j].y)) {
            return true;
        }
    }
    return PointInPolygon((x1 + x2) / 2, (y1 + y2) / 2, polygon);
}

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
