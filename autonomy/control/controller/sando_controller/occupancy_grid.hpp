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
 * @file occupancy_grid.hpp
 * @brief Local occupancy grid, heat field, and obstacle point queries.
 *
 * The grid is a cropped copy of Costmap2D, inflated by the robot radius.
 * Out-of-map reads are occupied, so a search cannot step off the copy.
 * Heat is a separate float layer used as an additive A* cost and as the
 * weight that decides which corners the path cleaner may cut.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "Eigen/Dense"

#include "autonomy/control/controller/sando_controller/types.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Occupancy label stored in one cell.
 *
 * Lethal and inscribed costmap costs become occupied. No-information becomes
 * unknown. Every other cost becomes free. Inflation then paints a disk of
 * occupied cells around each occupied seed.
 */
enum class Cell : uint8_t {
  kFree = 0,       ///< Traversable. Search may enter it.
  kUnknown = 1,    ///< No-information in the source costmap. Optional extra cost, and a safe-subgoal stop.
  kOccupied = 2,   ///< Lethal, inscribed, or inside the inflation radius. Search treats it as blocked.
};

/**
 * @class OccupancyGrid
 * @brief Row-major occupancy and heat copy focused on the robot.
 */
class OccupancyGrid {
 public:
  /**
   * @brief Rasterize a costmap window, then inflate occupied cells.
   *
   * When focus_radius is positive the copy is the intersection of the costmap
   * with a square of that half-size around the focus. A non-positive radius
   * copies the whole costmap. Inflation uses inflation_radius as a disk
   * radius in meters and does not change the resolution.
   *
   * @param costmap Source costmap. Lethal is 254, inscribed is 253, unknown is 255.
   * @param inflation_radius Robot footprint radius used to grow occupied cells, meters.
   * @param focus_x Window center x, meters. Ignored when focus_radius is not positive.
   * @param focus_y Window center y, meters.
   * @param focus_radius Half-size of the copied window, meters. Negative copies everything.
   */
  void LoadFromCostmap(const map::costmap_2d::Costmap2D& costmap, double inflation_radius, double focus_x = 0.0,
            double focus_y = 0.0, double focus_radius = -1.0);

  /** @brief Column count. Zero before the first successful load. */
  int GetWidth() const { return width_; }

  /** @brief Row count. Zero before the first successful load. */
  int GetHeight() const { return height_; }

  /** @brief Cell size, meters. Copied from the costmap. */
  double GetResolution() const { return resolution_; }

  /** @brief True when the grid has no cells. */
  bool IsEmpty() const { return width_ <= 0 || height_ <= 0; }

  /**
   * @brief Convert a world point to a cell index.
   * @param x World x, meters.
   * @param y World y, meters.
   * @param column_index Output column. Unchanged when the point is outside.
   * @param row_index Output row. Unchanged when the point is outside.
   * @return False when the point is outside the copied window.
   */
  bool ConvertWorldToCell(double x, double y, int* column_index, int* row_index) const;

  /**
   * @brief Convert a cell index to the world coordinate of its center.
   * @param column_index Column. Not range-checked.
   * @param row_index Row. Not range-checked.
   * @param x Output world x, meters.
   * @param y Output world y, meters.
   */
  void ConvertCellToWorld(int column_index, int row_index, double* x, double* y) const;

  /**
   * @brief Occupancy at a cell. Any index outside the grid is occupied.
   * @param column_index Column.
   * @param row_index Row.
   */
  Cell GetCell(int column_index, int row_index) const;

  /**
   * @brief Heat at a cell. Outside the grid, or before BuildHeat, the value is 0.
   * @param column_index Column.
   * @param row_index Row.
   */
  float GetCellHeat(int column_index, int row_index) const;

  /**
   * @brief Paint a disk free. Used to unblock the committed start so the footprint does not trap the search.
   * @param x Disk center x, meters.
   * @param y Disk center y, meters.
   * @param radius Disk radius, meters.
   */
  void FreeDisk(double x, double y, double radius);

  /**
   * @brief Paint a disk with one occupancy label. Predicted obstacle centers use kOccupied.
   * @param x Disk center x, meters.
   * @param y Disk center y, meters.
   * @param radius Disk radius, meters.
   * @param cell Label written into every cell whose center lies in the disk.
   */
  void MarkDisk(double x, double y, double radius, Cell cell);

  /**
   * @brief Fill the heat layer from static boundary halos and predicted movers.
   *
   * Static heat keeps the maximum boundary halo and is capped at static_hmax. Dynamic heat
   * is heat_alpha0 * (1 - ||q - c|| / R)^2 inside the reachable disk, plus
   * heat_alpha1 times the maximum over a constant-velocity tube of
   * exp(-t / (0.5 T)) * (1 - ||q - mu(t)|| / R(t))^2. R(t) grows as the base
   * radius plus obst_max_vel * t. The written cell is the maximum of the static
   * and dynamic values, under the same cap. Occupied cells are skipped.
   *
   * @param static_alpha Peak of one static halo.
   * @param static_rmax Halo radius, meters.
   * @param static_hmax Cap on the static field.
   * @param obstacles Every tracked mover. The caller decides which tracks to pass; this function does not filter by speed.
   * @param now Clock time, seconds.
   * @param prediction_horizon Horizon T over which movers are sampled, seconds.
   * @param heat_alpha0 Scale of the dynamic field at the obstacle.
   * @param heat_alpha1 Scale on the time-decaying tube. The written value is the reachable base plus this scale times the tube.
   * @param tube_radius Extra radius added to the obstacle half-extent before the speed term, meters.
   * @param obst_max_vel Speed used to grow the reachable radius, m/s.
   * @param focus_x Heat is written only inside this focus disk, center x.
   * @param focus_y Focus center y.
   * @param focus_radius Focus radius, meters.
   */
  void BuildHeat(double static_alpha, double static_rmax, double static_hmax,
                 const std::vector<DynObstacle>& obstacles, double now,
                 double prediction_horizon, double heat_alpha0, double heat_alpha1,
                 double tube_radius, double obst_max_vel, double focus_x, double focus_y,
                 double focus_radius);

  /**
   * @brief Append world points that the corridor decomposition treats as obstacles.
   *
   * The output is cleared first. Occupied cells inside the axis-aligned query
   * box are then emitted. Unknown cells are emitted only when include_unknown
   * is true. The caller expands the box by the map buffer before calling.
   *
   * @param first_corner_x First corner x of the query box, meters. Order relative to the second corner does not matter.
   * @param first_corner_y First corner y of the query box, meters.
   * @param second_corner_x Second corner x of the query box, meters.
   * @param second_corner_y Second corner y of the query box, meters.
   * @param include_unknown Also emit unknown cells.
   * @param points Output point cloud. Cleared, then filled. Required.
   */
  void CollectObstaclePoints(double first_corner_x, double first_corner_y, double second_corner_x,
                     double second_corner_y, bool include_unknown,
                     std::vector<Eigen::Vector2d>* points) const;

  /**
   * @brief Distance to the nearest occupied cell, meters.
   * @param x Query x, meters.
   * @param y Query y, meters.
   * @param search_radius Maximum search radius, meters. The return is at least this value when nothing is found.
   * @return Euclidean distance from the query to the nearest occupied cell center, meters.
   */
  double ComputeNearestOccupiedDistance(double x, double y, double search_radius) const;

  /**
   * @brief Distance to the nearest cell of one label, meters.
   * @param x Query x, meters.
   * @param y Query y, meters.
   * @param search_radius Maximum search radius, meters.
   * @param kind Label to search for.
   * @return Distance in meters, or search_radius when no such cell is inside the window.
   */
  double ComputeNearestDistance(double x, double y, double search_radius, Cell kind) const;

 private:
  /**
   * @brief Row-major index column_index + row_index * width.
   * @param column_index Column. The caller has already range-checked it.
   * @param row_index Row.
   */
  int ComputeLinearIndex(int column_index, int row_index) const { return column_index + row_index * width_; }

  int width_{0};                  ///< Columns in the copied window.
  int height_{0};                 ///< Rows in the copied window.
  double resolution_{0.05};              ///< Cell size, meters.
  double origin_x_{0.0};          ///< World x of the corner of cell (0, 0), meters.
  double origin_y_{0.0};          ///< World y of the corner of cell (0, 0), meters.
  std::vector<uint8_t> cells_;    ///< Occupancy stored as Cell values, row-major.
  std::vector<float> heat_;       ///< Heat layer, same shape as cells_. Zero until BuildHeat.
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
