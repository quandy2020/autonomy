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
 * @file region_segmenter.hpp
 * @brief Splits a coverage field into room-like regions for per-region
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_SEGMENTER_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_SEGMENTER_HPP_

#include <vector>

#include "autonomy/task/navigation/coverage/types.hpp"

namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::RegionSegmenter
 * @brief Cuts a coverage field, or an occupancy map, into room-sized regions.
 *
 * Every region is an exact subset of the input field, so the union of the
 * regions equals the original field and no two regions overlap. Fields2Cover
 * only exposes a difference operator, therefore the grid intersection is
 * computed as @c field - (field - cell).
 */
class RegionSegmenter {
 public:
  /**
   * @brief Grid partition size requested by the caller.
   */
  struct Options {
    // Number of horizontal regions (1..8).
    int columns{2};

    // Number of vertical regions (1..8).
    int rows{2};
  };

  RegionSegmenter() = delete;

  /**
   * @brief Occupancy-grid room extraction options.
   *
   * The passage width decides where two rooms separate: doorways narrower than
   * @c min_passage_width pinch the erosion shut and become room boundaries,
   * wider openings keep the rooms joined. @c clearance removes the strip the
   * robot body cannot enter, and the resulting wall band is later covered by a
   * headland pass.
   */
  struct MapOptions {
    // Cell value treated as an obstacle.
    int occupied_threshold{65};

    // Rooms smaller than this area in square metres are dropped.
    double min_area{1.0};

    // Upper bound on the number of extracted rooms.
    int max_regions{32};

    // Doorways narrower than this split rooms; wider openings stay joined.
    double min_passage_width{1.4};

    // Robot radius kept clear of walls when the regions are built.
    double clearance{0.4};
  };

  /**
   * @brief Extracts free-space rooms from an occupancy grid.
   *
   * Seeds are grown on an eroded copy of the free space so doorways pinch
   * shut, then the labels are grown back over the full free area. Cells that
   * are free but unreachable from any seed are attached to their nearest
   * neighbour so no traversable patch is lost.
   *
   * @param[in] map Occupancy grid in its own frame.
   * @param[in] options Threshold, passage width, clearance and area limits.
   * @return Room polygons in the map frame, largest first; empty when the map
   *         has no traversable area.
   */
  static std::vector<F2CCells> MapSplit(const proto::OccupancyMap& map,
                                       const MapOptions& options);

  /**
   * @brief Splits @p field into @c columns x @c rows rectangular regions.
   * @param[in] field Coverage field, possibly carrying holes.
   * @param[in] options Grid size; each side is clamped to 1..8.
   * @return Non-empty regions in row-major order.
   */
  static std::vector<F2CCells> GridSplit(const F2CCells& field,
                                        const Options& options);

  /**
   * @brief Total area of a cell group.
   * @param[in] cells Cells whose area is summed.
   * @return Area in square metres.
   */
  static double Area(const F2CCells& cells);
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_SEGMENTER_HPP_
