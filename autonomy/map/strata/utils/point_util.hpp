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
#include <vector>

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

bool PointInPolygon(double px, double py, const std::vector<LngLat>& polygon);
bool SegmentsIntersect(double ax, double ay, double bx, double by, double cx, double cy,
                       double dx, double dy);
bool LineSegmentIntersectsPolygon(double x1, double y1, double x2, double y2,
                                  const std::vector<LngLat>& polygon);

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
