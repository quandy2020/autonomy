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
 * @file hg_objective.cpp
 * @brief Fields2Cover hg objective implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/hg_obj/hg_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double HGObjective::ComputeCost(
    const F2CCell& total_cell, const F2CCell& rem_cell) {
  return ComputeCost(
      static_cast<F2CCells>(total_cell), static_cast<F2CCells>(rem_cell));
}
double HGObjective::ComputeCost(
    const F2CCell& total_cell, const F2CCells& rem_cell) {
  return ComputeCost(static_cast<F2CCells>(total_cell), rem_cell);
}
double HGObjective::ComputeCost(
    const F2CCells& total_cell, const F2CCell& rem_cell) {
  return ComputeCost(total_cell, static_cast<F2CCells>(rem_cell));
}
double HGObjective::ComputeCost(
    const F2CCells& total_cell, const F2CCells& rem_cell) {
  return 1 - total_cell.difference(rem_cell).Area() / total_cell.Area();
}

}  // namespace autonomy::planning::planner::fields2cover::obj

