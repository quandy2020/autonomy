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
 * @file boustrophedon_order.hpp
 * @brief Fields2Cover boustrophedon order.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

/**
 * @class autonomy::planning::planner::fields2cover::rp::BoustrophedonOrder
 * @brief Order swaths in a boustrophedon (back-and-forth) pattern.
 */
class BoustrophedonOrder : public SingleCellSwathsOrderBase {
 protected:
  /**
   * @brief Sort @p swaths into boustrophedon order.
   * @param[in,out] swaths Swaths to reorder in place.
   */
  void SortSwaths(F2CSwaths& swaths) const override;
};


}  // namespace autonomy::planning::planner::fields2cover::rp

