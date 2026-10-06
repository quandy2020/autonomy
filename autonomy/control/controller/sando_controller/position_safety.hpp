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
 * @file position_safety.hpp
 * @brief Point queries used by hover evasion.
 *
 * These tests answer whether a candidate hover point is already inside the
 * inflated footprint or inside the predicted reach of a mover. They do not
 * build a corridor.
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"
#include "autonomy/control/controller/sando_controller/types.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief True when the world point falls in an occupied cell.
 * @param grid Inflated occupancy. A point outside the grid is occupied.
 * @param x World x, meters.
 * @param y World y, meters.
 */
/**
 * @brief Occupancy and predicted-reach tests for one world point.
 */
class PositionSafety {
 public:
  bool IsCellOccupied(const OccupancyGrid& grid, double x, double y) const;

/**
 * @brief True when a mover or a nearby occupied cell reaches the query point.
 *
 * The nearest occupied cell inside hover_d_trigger counts as a threat.
 * Each obstacle is tested at its current center and, when lookahead is set,
 * at the constant-velocity center over the hover lookahead samples.
 * The threat radius is the obstacle radius plus the robot radius.
 *
 * @param grid Occupancy used for the nearest-occupied test.
 * @param options Hover trigger distance and lookahead.
 * @param obstacles Tracked movers.
 * @param x Query x, meters.
 * @param y Query y, meters.
 * @param lookahead Also test predicted centers when true.
 */
  bool IsPositionThreatened(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                            const std::vector<DynObstacle>& obstacles, double x, double y, bool lookahead) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
