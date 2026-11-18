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
 * @file single_cell_swaths_order_base.cpp
 * @brief Fields2Cover single cell swaths order base implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

F2CSwaths SingleCellSwathsOrderBase::GenSortedSwaths(
    const F2CSwaths& swaths, uint32_t variant) const {
  // Always Sort swaths to work with them in the same direction.
  F2CSwaths new_swaths = swaths.Clone();
  if (new_swaths.size() > 0) {
    new_swaths.Sort();
    this->ChangeStartPoint(new_swaths, variant);
    this->SortSwaths(new_swaths);
    new_swaths.ReverseDirOddSwaths();
  }
  return new_swaths;
}

void SingleCellSwathsOrderBase::ChangeStartPoint(
    F2CSwaths& swaths, uint32_t variant) const {
  if (variant & 1) {
    swaths.Reverse();
  }
  if (variant & 2) {
    swaths[0].Reverse();
  }
}



}  // namespace autonomy::planning::planner::fields2cover::rp

