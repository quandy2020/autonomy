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
 * @file route_method.hpp
 * @brief RouteMethod route strategy.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_METHOD_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_METHOD_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief RouteMethod adapted from Open Navigation coverage.
 */
class RouteMethod {
 public:
  virtual ~RouteMethod() = default;

  /**
   * @brief Plan an ordered route over the swaths.
   * @param travel_cells Border-sharing cells the inter-cell bridges may follow
   * @param swath_cells Cells the swaths were generated from (carved corridors)
   * @param swaths_by_cells Per-cell swaths, indexed like swath_cells
   * @param settings Fully-resolved RouteMode (server has already applied
   * defaults)
   * @param start_end_point Optional start/end point for the route (used by TSP
   * only)
   * @return Ordered route: swath groups plus any headland connections
   */
  virtual F2CRoute Plan(
      const F2CCells& travel_cells, const F2CCells& swath_cells,
      const F2CSwathsByCells& swaths_by_cells, const proto::RouteMode& settings,
      const std::optional<F2CPoint>& start_end_point = std::nullopt) = 0;
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_METHOD_HPP_
