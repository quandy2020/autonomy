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
 * @file route_geometry.hpp
 * @brief Preserve per-cell route geometry while joining coverage components.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GEOMETRY_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GEOMETRY_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

#define AWARN_UNUSED(...) \
  AWARN << "Coverage route used a checked geometry fallback"
/**
 * @brief Orders the swaths that belong to one cell into a route.
 * @param[in] cell Cell that bounds the travel area.
 * @param[in] swaths Swaths generated inside @p cell.
 * @param[in] turn_cost Relative cost of ending a pass versus travelling.
 * @param[in] route_type Route strategy selector (boustrophedon, TSP, ...).
 * @return Route covering every swath of the cell.
 */
F2CRoute RouteThroughCell(const Field& cell, const Swaths& swaths,
                          double turn_cost, const int& route_type);
F2CRoute StitchCellRoutes(const F2CCells&, const std::vector<F2CRoute>&,
                          const std::optional<F2CPoint>&, double, const int&);

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GEOMETRY_HPP_
