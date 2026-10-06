/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file position_safety.cpp
 * @brief Occupied-cell and predicted-threat tests for a single world point.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/position_safety.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

bool PositionSafety::IsCellOccupied(const OccupancyGrid& grid, double x, double y) const {
  int column_index = 0;
  int row_index = 0;
  return grid.ConvertWorldToCell(x, y, &column_index, &row_index) && grid.GetCell(column_index, row_index) == Cell::kOccupied;
}

bool PositionSafety::IsPositionThreatened(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                const std::vector<DynObstacle>& obstacles, double x, double y, bool lookahead) const {
  if (grid.ComputeNearestOccupiedDistance(x, y, options.hover_d_trigger()) < options.hover_d_trigger()) {
    return true;
  }
  const double horizon = lookahead ? options.hover_lookahead() : 0.0;
  for (const auto& obs : obstacles) {
    const double speed = std::hypot(obs.motion().velocity().linear().x(), obs.motion().velocity().linear().y());
    if (std::hypot(x - obs.motion().pose().position().x(), y - obs.motion().pose().position().y()) < options.hover_d_trigger() + obs.radius()) {
      return true;
    }
    if (!lookahead || speed < options.velocity_threshold()) {
      continue;
    }
    for (double t = 0.0; t <= horizon; t += 0.5) {
      const double px = obs.motion().pose().position().x() + obs.motion().velocity().linear().x() * t;
      const double py = obs.motion().pose().position().y() + obs.motion().velocity().linear().y() * t;
      if (std::hypot(x - px, y - py) < options.hover_d_trigger() + obs.radius()) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
