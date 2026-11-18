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
 * @file spiral_order.cpp
 * @brief Fields2Cover spiral order implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/route_planning/spiral_order.hpp"

namespace autonomy::planning::planner::fields2cover::rp {


SpiralOrder::SpiralOrder(size_t sp_size) {
  SetSpiralSize(static_cast<size_t>(std::max(2, static_cast<int>(sp_size))));
}

SpiralOrder::~SpiralOrder() = default;

void SpiralOrder::SetSpiralSize(size_t sp_size) {
  this->spiral_size = sp_size;
}

void SpiralOrder::SortSwaths(F2CSwaths& swaths) const {
  size_t spiral_count = swaths.size() / spiral_size;
  for (size_t i = 0; i < spiral_count; i++) {
    spiral(swaths, i * spiral_size, spiral_size);
  }

  int swaths_left = swaths.size() - spiral_count * spiral_size;
  if (swaths_left > 1) {
    spiral(swaths, spiral_count * spiral_size, swaths_left);
  }
}

void SpiralOrder::spiral(F2CSwaths& swaths, size_t offset, size_t size) const {
  for (size_t j = (offset + 1) % 2; j < size; j += 2) {
    std::rotate(swaths.begin() + offset + j,
                swaths.begin() + offset + size - 1,
                swaths.begin() + offset + size);
  }
}

}  // namespace autonomy::planning::planner::fields2cover::rp

