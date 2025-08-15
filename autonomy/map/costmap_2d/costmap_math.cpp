/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/map/costmap_2d/costmap_math.hpp"

#include <vector>
namespace autonomy {
namespace map {
namespace costmap_2d {

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
    double A = pX - x0;
    double B = pY - y0;
    double C = x1 - x0;
    double D = y1 - y0;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
        xx = x0;
        yy = y0;
    } else if (param > 1) {
        xx = x1;
        yy = y1;
    } else {
        xx = x0 + param * C;
        yy = y0 + param * D;
    }

    return distance(pX, pY, xx, yy);
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy