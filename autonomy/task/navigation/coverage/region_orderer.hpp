// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file region_orderer.hpp
 * @brief TSP ordering for coverage rooms. */
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace autonomy::task::coverage {

/**
 * @class RegionOrderer
 * @brief Orders coverage rooms for short inter-room travel.
 *
 * Nearest-neighbour construction followed by 2-opt refinement. Room counts are
 * small (a home map has tens of rooms), so an exact solver is unnecessary.
 * Distances are supplied by the caller so that wall-aware travel lengths can
 * replace straight-line distances.
 */
class RegionOrderer {
 public:
  /** @brief Ordered indices and the total travel length. */
  struct Result {
    std::vector<std::size_t> order;  /**< Visit order into the input list. */
    double distance{0.0};            /**< Total travel in metres. */
  };

  RegionOrderer() = delete;

  /**
   * @brief Orders @p centroids starting from (@p start_x, @p start_y).
   * @param centroids Room centroids in the map frame.
   * @param start_x Robot start x.
   * @param start_y Robot start y.
   * @return Visit order and total straight-line travel distance.
   */
  static Result Order(const std::vector<std::pair<double, double>>& centroids,
                      double start_x, double start_y);

  /**
   * @brief Orders rooms using an explicit travel-cost matrix.
   * @param costs @c costs[i][j] travel cost from room @c i to room @c j.
   * @param from_start Travel cost from the robot pose to each room.
   * @return Visit order and total travel cost.
   */
  static Result Order(const std::vector<std::vector<double>>& costs,
                      const std::vector<double>& from_start);
};

}  // namespace autonomy::task::coverage
