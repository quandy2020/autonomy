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
 * @file path_searcher.hpp
 * @brief Heat-weighted global path on the occupancy grid.
 *
 * Search is 8-connected A* with additive heat, unknown, and alignment
 * costs. Jump is a 2D jump-point search that still expands all
 * eight directions from every node. Both return a polyline of cell centers.
 * Shortcut and Prune are the HGP post-process: line of
 * sight, short-edge dropping, collinear cleanup, and heat-aware corner cuts.
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Weights and window for one search call.
 *
 * The window is an axis-aligned box in world coordinates. A failed search
 * retries with an unbounded window. reference_direction_x and reference_direction_y are a unit hint, usually
 * the first segment of the previous path; the alignment cost penalizes steps
 * that turn away from it.
 */
struct SearchWeights {
  double heuristic_weight{2.0};           ///< Multiplier on the Euclidean distance to the goal, meters.
  double heat_weight{10.0};               ///< Multiplier on GetCellHeat. Zero disables the heat cost.
  double unknown_cell_cost{0.0};             ///< Extra cost added when a step enters an unknown cell.
  double soft_occupied_cost{5.0};       ///< Extra cost for occupied cells when allow_soft_occupied is set.
  bool allow_soft_occupied{false}; ///< When true, occupied cells are expensive instead of blocked.
  int max_expansions{100000};      ///< Expansion budget. The search fails when this is exhausted.
  int timeout_milliseconds{300};             ///< Wall-clock budget. Checked every 64 A* expansions and every 32 JPS expansions.
  double alignment_weight{0.0};             ///< Weight on max(0, 1 - cos) between the step and the direction hint.
  double alignment_decay_cell_count{100.0};   ///< Alignment and crossing costs decay as exp(-distance_cells / alignment_decay_cell_count).
  double crossing_weight{0.0};              ///< Extra cost when the step crosses to the opposite side of the hint.
  double reference_direction_x{0.0};               ///< Direction hint, x. A zero vector disables alignment.
  double reference_direction_y{0.0};               ///< Direction hint, y.
  double window_min_x{-1.0e9};     ///< Inclusive minimum world x of the search window, meters.
  double window_min_y{-1.0e9};     ///< Inclusive minimum world y of the search window, meters.
  double window_max_x{1.0e9};      ///< Inclusive maximum world x of the search window, meters.
  double window_max_y{1.0e9};      ///< Inclusive maximum world y of the search window, meters.
};

/**
 * @brief Heat-weighted 8-connected A*.
 *
 * The step cost is the Euclidean length plus heat, unknown, soft-occupied,
 * and the alignment terms. The direction hint is flipped when it points away
 * from the goal. Diagonal steps are allowed. The path is a polyline of cell
 * centers from the start cell to the goal cell.
 *
 * @param grid Occupancy and heat. Out-of-grid cells are occupied.
 * @param start_x Start x, meters.
 * @param start_y Start y, meters.
 * @param goal_x Goal x, meters.
 * @param goal_y Goal y, meters.
 * @param weights Costs, budget, and window.
 * @param path Output polyline. Cleared and filled on success.
 * @return False when the start or goal is blocked, the budget expires, or no path exists.
 */
/**
 * @brief Heat-weighted A* and jump-point search, plus the geometric post-process.
 */
class PathSearcher {
 public:
  bool Search(const OccupancyGrid& grid, double start_x, double start_y, double goal_x, double goal_y,
                  const SearchWeights& weights, std::vector<Eigen::Vector2d>* path) const;

/**
 * @brief 2D jump-point search on the same cost model as Search.
 *
 * This is not pruned 3D JPS. Every node considers all eight directions, and
 * a diagonal jump also calls the two cardinal jumps. Unknown cells block the
 * jump. The caller falls back to Search when a jumped path still crosses
 * heat above 0.5.
 *
 * @param grid Occupancy and heat.
 * @param start_x Start x, meters.
 * @param start_y Start y, meters.
 * @param goal_x Goal x, meters.
 * @param goal_y Goal y, meters.
 * @param weights Costs, budget, and window.
 * @param path Output polyline of jump points, including the endpoints.
 * @return False on the same conditions as Search.
 */
  bool Jump(const OccupancyGrid& grid, double start_x, double start_y, double goal_x, double goal_y,
                        const SearchWeights& weights, std::vector<Eigen::Vector2d>* path) const;

/**
 * @brief Greedy line-of-sight shortcut, then drop short edges with a small turn.
 *
 * The line of sight is a capsule of line_of_sight_radius_cells. An edge shorter than minimum_edge_length
 * whose turn is below minimum_turn_degrees is removed. minimum_turn_degrees of 0 never drops
 * an edge, which is the filter-off setting.
 *
 * @param grid Occupancy used by the line-of-sight test.
 * @param path Polyline rewritten in place. A path with fewer than three points is unchanged.
 * @param line_of_sight_radius_cells Half-width of the clear capsule, in cells. Zero tests the center line only.
 * @param minimum_edge_length Minimum kept edge length, meters.
 * @param minimum_turn_degrees Minimum turn, degrees, below which a short edge may be dropped.
 */
  void Shortcut(const OccupancyGrid& grid, std::vector<Eigen::Vector2d>* path,
                          int line_of_sight_radius_cells, double minimum_edge_length,
                          double minimum_turn_degrees) const;

/**
 * @brief Remove collinear points, then cut corners whose heat stays under the peak relaxation.
 *
 * Collinear means the taxicab change of the middle point is below 1e-2.
 * Corner cuts run in both directions. A cut is kept when the peak heat on
 * the replacement segment is at most the peak on the original corner plus 0.50.
 * heat_weight of 0 skips the heat test and only removes collinear points.
 *
 * @param grid Heat field read by the corner test.
 * @param path Polyline rewritten in place.
 * @param heat_weight Passed through so a zero weight disables heat-aware cuts.
 */
  void Prune(const OccupancyGrid& grid, std::vector<Eigen::Vector2d>* path, double heat_weight) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
