// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file region_segmenter.hpp
 * @brief Splits a coverage field into room-like regions for per-region
 *        planning and colored visualization. */
#pragma once

#include <vector>

#include "autonomy/task/navigation/coverage/types.hpp"

namespace autonomy::task::coverage {

/**
 * @class RegionSegmenter
 * @brief Grid partition of a coverage field into independent regions.
 *
 * Each region is the exact intersection of the field (including holes) with
 * one cell of a columns x rows grid, so the union of the regions equals the
 * original field and no region overlaps another. Fields2Cover only exposes a
 * difference operator, so the intersection is computed as
 * @c field - (field - cell).
 */
class RegionSegmenter {
 public:
  /** @brief Grid size requested by the caller. */
  struct Options {
    int columns{2};  /**< Horizontal region count. */
    int rows{2};     /**< Vertical region count. */
  };

  RegionSegmenter() = delete;

  /** @brief Occupancy-grid room extraction options. */
  struct MapOptions {
    int occupied_threshold{65};    /**< Cell value treated as a wall. */
    double min_area{1.0};          /**< Drop rooms smaller than this in m2. */
    int max_regions{32};           /**< Keep at most this many rooms. */
    double min_passage_width{1.4}; /**< Doorways narrower than this split
                                        rooms; wider openings stay joined. */
  };

  /**
   * @brief Extracts free-space rooms from an occupancy grid.
   * @param map Raster map in its own frame.
   * @param options Threshold, minimum area and cap.
   * @return Room polygons in the map frame, largest first.
   */
  static std::vector<F2CCells> MapSplit(const proto::OccupancyMap& map,
                                       const MapOptions& options);

  /**
   * @brief Splits @p field into @c columns x @c rows regions.
   * @param field Coverage field with optional holes.
   * @param options Grid size (each side 1..8).
   * @return Non-empty regions in row-major order.
   */
  static std::vector<F2CCells> GridSplit(const F2CCells& field,
                                        const Options& options);

  /**
   * @brief Total area of a cell group.
   * @param cells Cells to measure.
   * @return Summed area in square metres.
   */
  static double Area(const F2CCells& cells);
};

}  // namespace autonomy::task::coverage
