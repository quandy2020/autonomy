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
 * @file n_swath_modified.cpp
 * @brief Fields2Cover n swath modified implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/n_swath_modified.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double NSwathModified::ComputeCost(
    double ang, double op_width, const F2CCell& cell) {
  if (cell.IsEmpty() || op_width <= 0.0) {
    return 0.0;
  }
  double n_turns {0.0};
  for (auto&& ring : cell) {
    for (size_t i = 0; i < ring.size(); ++i) {
      auto p = ring.GetGeometry(i);
      auto p_1 = ring.GetGeometry((i + ring.size() - 1) % ring.size());
      n_turns += p.Distance(p_1) *
        fabs(sin(ang - (p - p_1).GetAngleFromPoint()));
    }
  }
  return n_turns / (2.0 * op_width);
}

}  // namespace autonomy::planning::planner::fields2cover::obj

