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
 * @file trapezoidal_decomp.cpp
 * @brief Fields2Cover trapezoidal decomp implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/decomposition/trapezoidal_decomp.hpp"

namespace autonomy::planning::planner::fields2cover::decomp {

void TrapezoidalDecomp::SetSplitAngle(double ang) {
  this->split_angle = ang;
}

double TrapezoidalDecomp::GetSplitAngle() const {
  return this->split_angle;
}

F2CMultiLineString TrapezoidalDecomp::GenSplitLines(
    const F2CCells& cells, const obj::DecompObjective& obj) {
  F2CMultiLineString lines;
  for (auto&& cell : cells) {
    for (auto&& ring : cell) {
      for (auto&& p : ring) {
        lines.AddGeometry(cells.CreateLineUntilBorder(
            p, split_angle));
        lines.AddGeometry(cells.CreateLineUntilBorder(
            p, split_angle + M_PI));
      }
    }
  }
  return lines;
}

}  // namespace autonomy::planning::planner::fields2cover::decomp

