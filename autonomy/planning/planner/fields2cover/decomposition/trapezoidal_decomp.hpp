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
 * @file trapezoidal_decomp.hpp
 * @brief Fields2Cover trapezoidal decomp.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/decomposition/decomposition_base.hpp"

namespace autonomy::planning::planner::fields2cover::decomp {

/**
 * @class autonomy::planning::planner::fields2cover::decomp::TrapezoidalDecomp
 * @brief Trapezoidal decomposition \cite latombe1991exact.
 *
 * Each point Generate split lines perpendicular to the
 * \a split_angle.
 */
class TrapezoidalDecomp : public DecompositionBase {
 public:
  /**
   * @brief Get the split angle used to decompose the field.
   * @return Split angle in radians.
   */
  double GetSplitAngle() const;

  /**
   * @brief Set the split angle used to decompose the field.
   * @param[in] ang Split angle in radians.
   */
  void SetSplitAngle(double ang);

 public:
  /**
   * @brief Split the field into several cells that are easier to cover.
   * @param[in] cells Original cells
   * @return Smaller cells that compound the field
   */
  F2CMultiLineString GenSplitLines(const F2CCells& cells,
      const obj::DecompObjective& obj = obj::DecompObjective()) override;

 protected:
  double split_angle {0.0};
};


}  // namespace autonomy::planning::planner::fields2cover::decomp


