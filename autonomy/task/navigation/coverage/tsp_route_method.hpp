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
 * @file tsp_route_method.hpp
 * @brief TspRouteMethod route strategy.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_TSP_ROUTE_METHOD_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_TSP_ROUTE_METHOD_HPP_

#include "autonomy/task/navigation/coverage/route_method.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief TspRouteMethod adapted from Open Navigation coverage.
 */
class TspRouteMethod : public RouteMethod {
 public:
  // max_swaths_for_global_route: unused, kept for ABI/call-site stability
  TspRouteMethod(const int& logger, size_t max_swaths_for_global_route)
      : logger_(logger),
        max_swaths_for_global_route_(max_swaths_for_global_route) {}

  /**
   * @brief Return the complete computed path or plan the requested route.
   */
  F2CRoute Plan(
      const F2CCells& travel_cells, const F2CCells& swath_cells,
      const F2CSwathsByCells& swaths_by_cells, const proto::RouteMode& settings,
      const std::optional<F2CPoint>& start_end_point = std::nullopt) override;

 private:
  int logger_;
  size_t max_swaths_for_global_route_;
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_TSP_ROUTE_METHOD_HPP_
