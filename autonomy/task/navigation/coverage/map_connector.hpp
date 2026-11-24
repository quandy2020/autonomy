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
 * @file map_connector.hpp
 * @brief Free-space connectivity queries on an occupancy grid.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_MAP_CONNECTOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_MAP_CONNECTOR_HPP_

#include <utility>
#include <vector>

#include "autonomy/task/navigation/coverage/types.hpp"

namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::MapConnector
 * @brief Plans room-to-room links through the free space of an occupancy map.
 *
 * The raster is converted into an 8-connected graph once, at construction:
 * every cell within @c clearance of an obstacle is removed, so any path this
 * class returns can be driven by a robot of that radius. Distances are metric
 * (grid steps weighted by the cell size), which is what the room ordering needs
 * to avoid the straight-line shortcuts that would cut through walls.
 */
class MapConnector {
 public:
  /**
   * @brief Raster interpretation options.
   *
   * @c occupied_threshold decides which grid values count as obstacles and
   * @c clearance is the keep-out band applied around them, normally the robot
   * radius plus a small safety margin.
   */
  struct Options {
    Options() = default;

    /**
     * @brief Stores the interpretation explicitly.
     * @param[in] threshold Cell value treated as a wall.
     * @param[in] band Keep-out band in metres around every obstacle.
     */
    Options(int threshold, double band)
        : occupied_threshold(threshold), clearance(band) {}

    // Cell value treated as an obstacle.
    int occupied_threshold{65};

    // Keep-out band kept free around every obstacle, in metres.
    double clearance{0.0};
  };

  /**
   * @brief Result of one shortest-path query.
   *
   * @c points always contains both requested end points when a route exists, so
   * callers can emit segments without inserting extra vertices.
   */
  struct Visit {
    // World-frame waypoints from the start point to the goal point.
    std::vector<std::pair<double, double>> points;

    // Travelled length of @c points in metres.
    double distance{0.0};
  };

  /**
   * @brief Rasterises @p map with the default interpretation.
   * @param[in] map Occupancy grid to plan on.
   */
  inline MapConnector(const proto::OccupancyMap& map)
      : MapConnector(map, Options{}) {}

  /**
   * @brief Rasterises @p map and removes the clearance band.
   * @param[in] map Occupancy grid to plan on.
   * @param[in] options Threshold and keep-out band.
   */
  MapConnector(const proto::OccupancyMap& map, const Options& options);

  /**
   * @brief Reports whether the raster holds any traversable cell.
   * @return true when the grid is non-empty and at least one cell is free.
   */
  bool Valid() const { return width_ > 0 && height_ > 0 && !free_.empty(); }

  /**
   * @brief Tests one world point against the inflated raster.
   * @param[in] x World x coordinate in metres.
   * @param[in] y World y coordinate in metres.
   * @return true when a robot centred there clears every obstacle.
   */
  bool Free(double x, double y) const;

  /**
   * @brief Shortest traversable link between two world points.
   *
   * Collinear stretches are collapsed, so the returned polyline only carries
   * the corners a follower has to steer through.
   *
   * @param[in] from Start point in the map frame.
   * @param[in] to Goal point in the map frame.
   * @return Waypoints including both ends; @c points is empty when no route
   *         connects the snapped start and goal cells.
   */
  Visit Connect(const std::pair<double, double>& from,
                const std::pair<double, double>& to) const;

  /**
   * @brief Shortest traversable distance between two world points.
   * @param[in] from Start point in the map frame.
   * @param[in] to Goal point in the map frame.
   * @return Path length in metres, or infinity when unreachable.
   */
  double Distance(const std::pair<double, double>& from,
                  const std::pair<double, double>& to) const;

  /**
   * @brief Distances to many targets from one source using a single flood.
   *
   * Ordering a room tour needs a full distance matrix. Running one Dijkstra per
   * pair is quadratic in the room count and dominated the plan time on large
   * maps, so the flood stops as soon as every requested target is settled.
   *
   * @param[in] from Start point in the map frame.
   * @param[in] targets Goal points, indexed by the caller.
   * @return One distance per entry of @p targets in metres; infinity marks a
   *         target that cannot be reached from @p from.
   */
  std::vector<double> Distances(
      const std::pair<double, double>& from,
      const std::vector<std::pair<double, double>>& targets) const;

  /**
   * @brief Closest traversable point to @p point.
   * @param[in] point Query point in the map frame.
   * @return The snapped point, or @p point itself when the raster is invalid.
   */
  std::pair<double, double> NearestFree(
      const std::pair<double, double>& point) const;

 private:
  /**
   * @brief Tests one raster cell.
   * @param[in] column Raster column index.
   * @param[in] row Raster row index.
   * @return true when the cell is inside the grid and traversable.
   */
  bool CellFree(int column, int row) const;

  /**
   * @brief Converts a world point to a raster cell.
   * @param[in] x World x coordinate in metres.
   * @param[in] y World y coordinate in metres.
   * @return Column and row of the containing cell, possibly outside the grid.
   */
  std::pair<int, int> Cell(double x, double y) const;

  /**
   * @brief Converts a raster cell to its centre in the map frame.
   * @param[in] column Raster column index.
   * @param[in] row Raster row index.
   * @return Cell centre in metres.
   */
  std::pair<double, double> World(int column, int row) const;

  /**
   * @brief Removes every cell closer than @p clearance to an obstacle.
   * @param[in] clearance Keep-out band in metres.
   */
  void Inflate(double clearance);

  // Raster width in cells.
  int width_{0};

  // Raster height in cells.
  int height_{0};

  // Metres per cell.
  double resolution_{0.0};

  // Map-frame x of the first cell corner.
  double origin_x_{0.0};

  // Map-frame y of the first cell corner.
  double origin_y_{0.0};

  // Rotation of the raster in the map frame, radians.
  double origin_yaw_{0.0};

  // Traversable mask after inflation (1 = free).
  std::vector<unsigned char> free_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_MAP_CONNECTOR_HPP_
