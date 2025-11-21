/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "autonomy/localization/amcl/map/map.hpp"

namespace autonomy {
namespace localization {
namespace amcl {
namespace map {

// // Extract a single range reading from the map.  Unknown cells and/or
// // out-of-bound cells are treated as occupied, which makes it easy to
// // use Stage bitmap files.
// double map_calc_range(map_t * map, double ox, double oy, double oa, double max_range)
// {
//   // Bresenham raytracing
//   int x0, x1, y0, y1;
//   int x, y;
//   int xstep, ystep;
//   char steep;
//   int tmp;
//   int deltax, deltay, error, deltaerr;

//   x0 = MAP_GXWX(map, ox);
//   y0 = MAP_GYWY(map, oy);

//   x1 = MAP_GXWX(map, ox + max_range * cos(oa));
//   y1 = MAP_GYWY(map, oy + max_range * sin(oa));

//   if (abs(y1 - y0) > abs(x1 - x0)) {
//     steep = 1;
//   } else {
//     steep = 0;
//   }

//   if (steep) {
//     tmp = x0;
//     x0 = y0;
//     y0 = tmp;

//     tmp = x1;
//     x1 = y1;
//     y1 = tmp;
//   }

//   deltax = abs(x1 - x0);
//   deltay = abs(y1 - y0);
//   error = 0;
//   deltaerr = deltay;

//   x = x0;
//   y = y0;

//   if (x0 < x1) {
//     xstep = 1;
//   } else {
//     xstep = -1;
//   }
//   if (y0 < y1) {
//     ystep = 1;
//   } else {
//     ystep = -1;
//   }

//   if (steep) {
//     if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1) {
//       return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
//     }
//   } else {
//     if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1) {
//       return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
//     }
//   }

//   while (x != (x1 + xstep * 1)) {
//     x += xstep;
//     error += deltaerr;
//     if (2 * error >= deltax) {
//       y += ystep;
//       error -= deltax;
//     }

//     if (steep) {
//       if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1) {
//         return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
//       }
//     } else {
//       if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1) {
//         return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
//       }
//     }
//   }
//   return max_range;
// }

}   // namespace map
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy