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
 * @file decomposition_base.hpp
 * @brief Fields2Cover decomposition base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/decomp_obj/decomp_objective.hpp"

namespace autonomy::planning::planner::fields2cover::decomp {

/**
 * @class autonomy::planning::planner::fields2cover::decomp::DecompositionBase
 *  @brief Base class for algorithms that decompose non-convex cells
 * into multiple simpler cells.
 *
 * The class has one main method (autonomy::planning::planner::fields2cover::decomp::decompose) that split
 * complex-shape cells into simpler cells.
 */
class DecompositionBase {
 public:
  /**
   * @brief Decompose F2CCells into simpler F2CCells.
   *
   * This method decompose every Cell in @p cells into simpler cells.
   * @param[in] cells Complex-shape cells
   * @return Same space as @p cells recoded into simpler cells
   */
  virtual F2CCells decompose(const F2CCells& cells,
      const obj::DecompObjective& obj = obj::DecompObjective());

 public:
  /**
   * @brief Split the field into several cells that are easier to cover.
   * @param[in] cells Original cells
   * @return Smaller cells that compound the field
   */
  virtual F2CCells split(const F2CCells& cells,
      const obj::DecompObjective& obj);

  /**
   * @brief Generate the lines used by autonomy::planning::planner::fields2cover::decomp::split to split the cells into
   * simpler cells
   * @param[in] cells Complex-shape cells
   * @return Lines that split the complex-shape cells into simpler cells
   */
  virtual F2CMultiLineString GenSplitLines(const F2CCells& cells,
      const obj::DecompObjective& obj) = 0;

  /**
   * @brief Apply a merge strategy to reduce the number of simpler cells.
   * @param[in] cells Simple cells
   * @return Merged cells.
   */
  virtual F2CCells merge(const F2CCells& cells,
      const obj::DecompObjective& obj);

  virtual ~DecompositionBase() = default;
};

}  // namespace autonomy::planning::planner::fields2cover::decomp


