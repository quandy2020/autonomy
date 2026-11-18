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
 * @file single_cell_swaths_order_base.hpp
 * @brief Fields2Cover single cell swaths order base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

/**
 * @class autonomy::planning::planner::fields2cover::rp::SingleCellSwathsOrderBase
 * @brief Base class for ordering swaths that belong to a single cell.
 */
class SingleCellSwathsOrderBase {
 public:
  /**
   * @brief Return a copy of @p swaths sorted by this orderer.
   * @param[in] swaths Input swaths.
   * @param[in] variant Optional start-point / direction variant.
   * @return Sorted swaths.
   */
  virtual F2CSwaths GenSortedSwaths(
      const F2CSwaths& swaths, uint32_t variant = 0) const;

  /**
   * @brief Destructor.
   */
  virtual ~SingleCellSwathsOrderBase() = default;

 protected:
  /**
   * @brief Flip start/end of the first swath according to @p variant.
   * @param[in,out] swaths Swaths whose endpoints may be reversed.
   * @param[in] variant Start-point / direction variant.
   */
  virtual void ChangeStartPoint(F2CSwaths& swaths, uint32_t variant) const;

  /**
   * @brief Sort @p swaths in place.
   * @param[in,out] swaths Swaths to reorder.
   */
  virtual void SortSwaths(F2CSwaths& swaths) const = 0;
};



}  // namespace autonomy::planning::planner::fields2cover::rp

