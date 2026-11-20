// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_obstacles.hpp
 * @brief Live occupancy-grid obstacle extraction for the coverage panel. */
#pragma once

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace autoviz::navigation {

/** @brief One obstacle footprint in the map frame. */
struct ObstacleRing {
  /** @brief Ring vertices in metres, ordered counter-clockwise. */
  std::vector<std::pair<double, double>> points;
  /** @brief Number of occupied cells that formed this footprint. */
  std::size_t cells{0};
};

/** @brief Tuning for @ref CoverageObstacleExtractor. */
struct ObstacleExtractionOptions {
  /** @brief Cell value treated as occupied (costmap lethal threshold). */
  int lethal_threshold{65};
  /** @brief Extra clearance around every footprint, in metres. */
  double margin{0.2};
  /** @brief Components larger than this are static walls and are skipped. */
  std::size_t max_component_cells{2500};
  /** @brief Upper bound on emitted footprints, largest first. */
  std::size_t max_rings{48};
};

/**
 * @class CoverageObstacleExtractor
 * @brief Converts occupied occupancy-grid cells into coverage exclusion rings.
 *
 * The coverage planner takes obstacles as polygon rings, while a live map
 * exposes them as raster cells. This class flood fills connected occupied
 * cells, keeps the compact blobs that represent movable obstacles, discards
 * long static walls, and returns an oriented bounding rectangle per blob with
 * the configured clearance margin.
 */
class CoverageObstacleExtractor {
 public:
  CoverageObstacleExtractor() = delete;

  /**
   * @brief Extracts obstacle footprints from an occupancy grid.
   * @param grid Full occupancy grid in its own frame.
   * @param options Occupancy threshold, margin and size limits.
   * @return Footprints sorted by cell count, at most @c max_rings entries.
   */
  static std::vector<ObstacleRing> Extract(
      const automsgs::msgs::map_msgs::OccupancyGrid& grid,
      const ObstacleExtractionOptions& options = {});

  /**
   * @brief Hashes the occupied cells so callers can detect map changes.
   * @param grid Full occupancy grid.
   * @param lethal_threshold Cell value treated as occupied.
   * @return Stable signature for the current obstacle raster.
   */
  static std::uint64_t Signature(
      const automsgs::msgs::map_msgs::OccupancyGrid& grid,
      int lethal_threshold);
};

}  // namespace autoviz::navigation
