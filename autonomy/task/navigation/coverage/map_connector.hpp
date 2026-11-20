// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file map_connector.hpp
 * @brief Free-space connectivity queries on an occupancy grid. */
#pragma once

#include <utility>
#include <vector>

#include "autonomy/task/navigation/coverage/types.hpp"

namespace autonomy::task::coverage {

/**
 * @class MapConnector
 * @brief Plans room-to-room links through the free space of an occupancy map.
 *
 * The raster is turned into a graph once: cells closer to a wall than
 * @c clearance are removed, so a plan produced here is traversable by a robot
 * of that radius. Shortest paths are computed with Dijkstra on the 8-connected
 * grid, which gives metric distances instead of the straight-line distances
 * that would cut through walls.
 */
class MapConnector {
 public:
  /** @brief Raster interpretation options. */
  struct Options {
    Options() = default;
    Options(int threshold, double band)
        : occupied_threshold(threshold), clearance(band) {}
    int occupied_threshold{65};  /**< Cell value treated as a wall. */
    double clearance{0.0};       /**< Keep-out band around obstacles. */
  };

  /** @brief Grid index and metric distance of one shortest-path query. */
  struct Visit {
    std::vector<std::pair<double, double>> points;  /**< World-frame waypoints. */
    double distance{0.0};                           /**< Path length in metres. */
  };

  /** @brief Rasterise @p map and inflate obstacles by the clearance band. */
  inline MapConnector(const proto::OccupancyMap& map)
      : MapConnector(map, Options{}) {}
  MapConnector(const proto::OccupancyMap& map, const Options& options);

  /** @brief Whether the raster produced at least one traversable cell. */
  bool Valid() const { return width_ > 0 && height_ > 0 && !free_.empty(); }

  /** @brief Whether a world point is traversable for the configured radius. */
  bool Free(double x, double y) const;

  /**
   * @brief Shortest traversable link between two world points.
   * @return Waypoints including both ends, or empty when unreachable.
   */
  Visit Connect(const std::pair<double, double>& from,
                const std::pair<double, double>& to) const;

  /** @brief Shortest traversable distance, or infinity when unreachable. */
  double Distance(const std::pair<double, double>& from,
                  const std::pair<double, double>& to) const;

  /** @brief Closest traversable point to @p point, or itself when invalid. */
  std::pair<double, double> NearestFree(
      const std::pair<double, double>& point) const;

 private:
  bool CellFree(int column, int row) const;
  std::pair<int, int> Cell(double x, double y) const;
  std::pair<double, double> World(int column, int row) const;
  void Inflate(double clearance);

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  std::vector<unsigned char> free_;
};

}  // namespace autonomy::task::coverage
