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

#pragma once

/**
 * @file boustrophedon_decomp.hpp
 * @brief Fields2Cover boustrophedon decomp.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/decomposition/trapezoidal_decomp.hpp"

namespace autonomy::planning::planner::fields2cover::decomp {

/**
 * @class autonomy::planning::planner::fields2cover::decomp::BoustrophedonDecomp
 * @brief Boustrophedon Cellular decomposition \cite choset2000coverage.
 *
 * Each point in the geometry is checked if it is a critical point.
 * A critical point is defined as a point that split or merge cells
 * in the \a split_angle .
 * If it is a critical point, split lines are created in the perpendicular
 * to the \a split_angle.
 */
class BoustrophedonDecomp : public TrapezoidalDecomp {
 public:
  /**
   * @brief Split the field into several cells that are easier to cover.
   * @param[in] cells Original cells
   * @return Smaller cells that compound the field
   */
  F2CMultiLineString GenSplitLines(const F2CCells& cells,
      const obj::DecompObjective& obj = obj::DecompObjective()) override;

 private:
  bool IsLinePartOfLinearRing(const F2CLinearRing & ring, const F2CLineString & line) const;
};

}  // namespace autonomy::planning::planner::fields2cover::decomp


