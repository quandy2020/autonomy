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
 * @file custom_order.hpp
 * @brief Fields2Cover custom order.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include <set>
#include <string>
#include <stdexcept>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"


namespace autonomy::planning::planner::fields2cover::rp {

/**
 * @class autonomy::planning::planner::fields2cover::rp::CustomOrder
 * @brief Order swaths using an explicit index sequence.
 */
class CustomOrder : public SingleCellSwathsOrderBase {
 public:
  /**
   * @brief Construct with a custom swath index order.
   * @param[in] order Indices into the unsorted swath list.
   */
  explicit CustomOrder(
      const std::vector<size_t>& order = std::vector<size_t>());

  /**
   * @brief Destructor.
   */
  ~CustomOrder();

  /**
   * @brief Replace the stored custom order.
   * @param[in] order Indices into the unsorted swath list.
   */
  void SetCustomOrder(const std::vector<size_t>& order);

 protected:
  /**
   * @brief Sort @p swaths to match the custom index order.
   * @param[in,out] swaths Swaths to reorder in place.
   */
  void SortSwaths(F2CSwaths& swaths) const override;

 private:
  std::vector<size_t> custom_order;

  /**
   * @brief Validate that @p order is a permutation of @p swaths indices.
   * @param[in] swaths Swaths that will be reordered.
   */
  void check(const F2CSwaths& swaths) const;
};


}  // namespace autonomy::planning::planner::fields2cover::rp

