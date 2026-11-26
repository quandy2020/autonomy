/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <cstdint>

namespace automsgs {
namespace msgs {
namespace map_msgs {

// ROS nav_msgs/OccupancyGrid int8[] range: -1 unknown, 0–100 occupancy.
inline int OccupancyCellToInt8(int value) {
  if (value < 0 || value > 100) {
    return -1;
  }
  return value;
}

template <typename GridT>
int OccupancyGridCellCount(const GridT& grid) {
  return grid.data_size();
}

template <typename GridT>
int OccupancyGridCell(const GridT& grid, int index) {
  if (index < 0 || index >= grid.data_size()) {
    return -1;
  }
  const int value = grid.data(index);
  // ROS nav_msgs/OccupancyGrid is int8[]: -1 unknown, 0–100 occupancy.
  // Some bridges store unknown as uint8 255 instead of signed -1.
  if (value == -1 || value == 255) {
    return -1;
  }
  return static_cast<int>(
      static_cast<int8_t>(std::clamp(value, -128, 127)));
}

template <typename GridT>
void OccupancyGridClearCells(GridT* grid) {
  grid->clear_data();
}

template <typename GridT>
void OccupancyGridReserveCells(GridT* grid, int count) {
  grid->mutable_data()->Reserve(std::max(0, count));
}

template <typename GridT>
void OccupancyGridResizeCells(GridT* grid, int count, int fill = 0) {
  grid->mutable_data()->Resize(std::max(0, count), OccupancyCellToInt8(fill));
}

template <typename GridT>
void OccupancyGridSetCell(GridT* grid, int index, int value) {
  if (index < 0 || index >= grid->data_size()) {
    return;
  }
  grid->set_data(index, OccupancyCellToInt8(value));
}

template <typename GridT>
void OccupancyGridAppendCell(GridT* grid, int value) {
  grid->add_data(OccupancyCellToInt8(value));
}

}  // namespace map_msgs
}  // namespace msgs
}  // namespace automsgs
