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
 * @file constant_headland.cpp
 * @brief Fields2Cover constant headland implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <utility>
#include "autonomy/planning/planner/fields2cover/headland_generator/constant_headland.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

F2CCells ConstHL::GenerateHeadlands(
    const F2CCells& field, double dist_headland) {
  return field.buffer(-dist_headland);
}

F2CCells ConstHL::GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) {
  return field.buffer(-swath_width * n_swaths);
}

std::vector<F2CCells> ConstHL::GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths, bool dir_out2in) {
  std::vector<F2CCells> hl;
  if (dir_out2in) {
    for (int i = 0; i < n_swaths; ++i) {
      hl.emplace_back(field.buffer(-swath_width * (i + 0.5)));
    }
  } else {
    hl.emplace_back(field.buffer(-swath_width * (n_swaths - 0.5)));
    for (int i = 1; i < n_swaths; ++i) {
      hl.emplace_back(hl.back().buffer(swath_width));
    }
  }
  return hl;
}


}  // namespace autonomy::planning::planner::fields2cover::hg

