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
 * @file spiral_order.hpp
 * @brief Fields2Cover spiral order.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

/**
 * @class autonomy::planning::planner::fields2cover::rp::SpiralOrder
 * @brief Order swaths in a spiral pattern inside one cell.
 */
class SpiralOrder : public SingleCellSwathsOrderBase {
 public:
  /**
   * @brief Construct with a spiral window size.
   * @param[in] sp_size Number of swaths in each spiral cycle (min 2).
   */
  explicit SpiralOrder(size_t sp_size = 2);

  /**
   * @brief Destructor.
   */
  ~SpiralOrder();

  /**
   * @brief Set the spiral window size.
   * @param[in] sp_size Number of swaths in each spiral cycle.
   */
  void SetSpiralSize(size_t sp_size);

 protected:
  /**
   * @brief Sort @p swaths into spiral order.
   * @param[in,out] swaths Swaths to reorder in place.
   */
  void SortSwaths(F2CSwaths& swaths) const override;

 private:
  size_t spiral_size;

  /**
   * @brief Apply one spiral cycle of length @p size starting at @p offset.
   * @param[in,out] swaths Swaths being reordered.
   * @param[in] offset First index of this cycle.
   * @param[in] size Cycle length.
   */
  void spiral(F2CSwaths& swaths, size_t offset, size_t size) const;
};


}  // namespace autonomy::planning::planner::fields2cover::rp

