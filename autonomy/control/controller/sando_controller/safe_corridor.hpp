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
 * @file safe_corridor.hpp
 * @brief Time-layered polyhedral safe corridor along a geometric path.
 *
 * Each spatial segment is decomposed with an ellipsoid (LineSegment) whose
 * supporting half-planes become the polytope. The ellipsoid itself is not a
 * constraint. Time is a stack of layers, not a fourth ellipsoid axis: layer n
 * is the free space at time (n + 1) * dt. A static scene copies the first
 * layer. A dynamic scene rebuilds the obstacle cloud at each layer from the
 * the current obstacle centers, expanded by obst_max_vel * t.
 */

#pragma once

#include <vector>

#include "Eigen/Dense"

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"
#include "autonomy/control/controller/sando_controller/types.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Build layers[time][spatial polytope] of half-planes.
 *
 * For each segment the decomposition seed is the segment inflated by
 * sfc_bbox. Half-planes are shrunk by robot_radius * ||n|| so the constraint
 * is on the robot center. Dynamic obstacles contribute a grid of points on
 * the rectangle grown by obst_max_vel * t + comm_delay * speed. Unknown
 * boundary cells, when inflate_unknown is set and t_end > 0, are inflated by
 * the L-infinity ball of radius obst_max_vel * t_end. An empty polytope is left empty and
 * the MIQP fixes the corresponding binary to zero.
 *
 * @param grid Occupancy and the static point cloud.
 * @param options Inflation, bbox, unknown-inflation, and dynamic-obstacle limits.
 * @param obstacles Tracked movers evaluated at each layer time.
 * @param spatial Path knots. Segment i joins spatial[i] and spatial[i + 1].
 * @param time_layers Number of time layers N. One layer per polynomial piece.
 * @param layer_duration Layer duration, seconds. Layer n is evaluated at (n + 1) * layer_duration.
 * @param layers Output. Resized to time_layers. Required.
 * @return False when the path has fewer than two points or a decomposition throws.
 */
/**
 * @brief Builds the time-layered polyhedral corridor consumed by the MIQP.
 */
class SafeCorridor {
 public:
  bool Build(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                     const std::vector<DynObstacle>& obstacles,
                     const std::vector<Eigen::Vector2d>& spatial, int time_layers, double layer_duration,
                     std::vector<std::vector<std::vector<HalfPlane>>>* layers) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
