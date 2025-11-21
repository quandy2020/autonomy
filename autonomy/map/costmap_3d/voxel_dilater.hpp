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

namespace autonomy {
namespace map {
namespace costmap_3d {

#ifndef VOXEL_DILATER
#define VOXEL_DILATER(i, j, k, x, y, z, sy, sz, bx, by, bz, ck, ogm, ofst, val, fdl)                                                                                                                                                      \
(ck) = (x) == 0 || (x) == (bx) || (y) == 0 || (y) == (by) || (z) == 0 || (z) == (bz);                                                                                                                                                   \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                                   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                                )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }
#endif

}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy