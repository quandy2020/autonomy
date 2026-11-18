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
 * @file snake_order.cpp
 * @brief Fields2Cover snake order implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/route_planning/snake_order.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

void SnakeOrder::SortSwaths(F2CSwaths& swaths) const {
  size_t i;
  for (i = 1; i < (swaths.size() - 1) / 2 + 1; ++i) {
    std::rotate(swaths.begin() + i, swaths.begin() + i + 1, swaths.end());
  }
  std::reverse(swaths.begin() + i + 1, swaths.end());
  if (swaths.size() % 2 == 1) {
    std::rotate(swaths.begin() + i, swaths.begin() + i + 1, swaths.end());
  }
}


}  // namespace autonomy::planning::planner::fields2cover::rp
