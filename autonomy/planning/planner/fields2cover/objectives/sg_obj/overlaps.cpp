/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file overlaps.cpp
 * @brief Fields2Cover overlaps implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/overlaps.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double Overlaps::ComputeCost(const F2CCell& poly, const F2CSwaths& swaths) {
  return ComputeCost(F2CCells(poly), swaths);
}

double Overlaps::ComputeCost(const F2CCells& poly, const F2CSwaths& swaths) {
  double area_overlaped {0.0};
  std::vector<F2CCells> areas;
  for (int i = swaths.size() - 1; i >= 0; --i) {
    areas.emplace_back(swaths[i].AreaCovered(poly));
    for (int j = i + 1; j < areas.size(); ++j) {
      if (areas[i].intersects(areas[j])) {
        area_overlaped += F2CCells(areas[i]).intersection(areas[j]).Area();
      }
    }
  }
  return area_overlaped / poly.Area();
}


}  // namespace autonomy::planning::planner::fields2cover::obj

