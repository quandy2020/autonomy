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
 * @file custom_order.cpp
 * @brief Fields2Cover custom order implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/route_planning/custom_order.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

CustomOrder::CustomOrder(const std::vector<size_t>& order) :
  custom_order(order) {
}

CustomOrder::~CustomOrder() = default;

void CustomOrder::SetCustomOrder(const std::vector<size_t>& order) {
  custom_order = order;
}


void CustomOrder::SortSwaths(F2CSwaths& swaths) const {
  check(swaths);
  F2CSwaths sorted_swaths(swaths.size());
  for (size_t i = 0; i < custom_order.size(); ++i) {
    sorted_swaths[i] = swaths[custom_order[i]];
  }
  swaths = std::move(sorted_swaths);
}

void CustomOrder::check(const F2CSwaths& swaths) const {
  // unique vector
  std::vector<size_t> unique(custom_order.begin(), custom_order.end());
  // Sort the unique vector
  std::sort(unique.begin(), unique.end());
  auto last = std::unique(unique.begin(), unique.end());
  unique.erase(last, unique.end());
  // check the sizes
  if (unique.size() != custom_order.size()) {
    throw std::invalid_argument(
        "Order vector does not contain unique elements.");
  }

  if (swaths.size() != custom_order.size()) {
    throw std::length_error(
        "Lengths of the order vector and swaths must be the same.");
  }
  auto max_el = *std::max_element(custom_order.begin(), custom_order.end());
  if (max_el > swaths.size() - 1) {
    throw std::invalid_argument( \
        "Custom order element is out of the swath range [" + \
        std::to_string(max_el) + "]");
  }
}

}  // namespace autonomy::planning::planner::fields2cover::rp
