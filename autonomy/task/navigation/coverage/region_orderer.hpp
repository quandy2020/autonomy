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
 * @file region_orderer.hpp
 * @brief TSP ordering for coverage rooms.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_ORDERER_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_ORDERER_HPP_

#include <cstddef>
#include <utility>
#include <vector>

namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::RegionOrderer
 * @brief Orders the coverage rooms to keep the travel between them short.
 *
 * The tour is built with a nearest-neighbour construction and then improved by
 * 2-opt passes. A household map holds tens of rooms, so an exact solver buys
 * nothing while costing planning time. The caller supplies the cost matrix, so
 * the ordering can use wall-aware A* distances instead of straight lines.
 */
class RegionOrderer {
 public:
  /**
   * @brief Result of one ordering pass.
   */
  struct Result {
    // Visit order as indices into the caller's room list.
    std::vector<std::size_t> order;

    // Travelled length of @c order in metres.
    double distance{0.0};
  };

  RegionOrderer() = delete;

  /**
   * @brief Orders rooms by straight-line distance from one start pose.
   *
   * Convenience overload for callers without a map; it assumes the path between
   * two rooms is unobstructed, so prefer the cost-matrix overload whenever an
   * occupancy grid is available.
   *
   * @param[in] centroids Room centroids in the map frame.
   * @param[in] start_x Robot start x coordinate in metres.
   * @param[in] start_y Robot start y coordinate in metres.
   * @return Visit order and the summed straight-line travel distance.
   */
  static Result Order(const std::vector<std::pair<double, double>>& centroids,
                      double start_x, double start_y);

  /**
   * @brief Orders rooms using an explicit travel-cost matrix.
   *
   * @param[in] costs Asymmetric cost matrix; @c costs[i][j] is the traversable
   *            distance from room @c i to room @c j in metres, infinity when the
   *            two rooms cannot be connected.
   * @param[in] from_start Cost from the robot pose to every room, used to pick
   *            the first room of the tour.
   * @return Visit order and the summed travel cost of that order.
   */
  static Result Order(const std::vector<std::vector<double>>& costs,
                      const std::vector<double>& from_start);
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_REGION_ORDERER_HPP_
