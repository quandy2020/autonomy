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
 * @file tsp_route_method.cpp
 * @brief Native complete-coverage tsp route method implementation.
 */

#include "autonomy/task/navigation/coverage/tsp_route_method.hpp"

#include "autonomy/task/navigation/coverage/route_geometry.hpp"
namespace autonomy::task::coverage {
F2CRoute TspRouteMethod::Plan(const F2CCells& travel_cells,
                              const F2CCells& swath_cells,
                              const F2CSwathsByCells& swaths_by_cells,
                              const proto::RouteMode& settings,
                              const std::optional<F2CPoint>& start_end) {
  const bool redirect_swaths = settings.tsp_redirect_swaths();
  const long int time_limit = settings.tsp_time_limit();  // NOLINT
  const bool search_for_optimum = settings.tsp_search_for_optimum();
  const double d_tol = settings.tsp_d_tol();

  // Single cell: one genRoute call, which honors the start/end point exactly.
  if (swath_cells.size() <= 1) {
    f2c::rp::RoutePlannerBase rp;
    if (start_end) {
      rp.SetStartAndEndPoint(*start_end);
    }
    return rp.GenRoute(swath_cells, swaths_by_cells, false, d_tol,
                       redirect_swaths, time_limit, search_for_optimum);
  }

  // Multi-cell: solve each cell alone to avoid interleaving and the all-pairs
  // path matrix.
  std::vector<F2CRoute> cell_routes(swath_cells.size());
  for (size_t i = 0; i < swath_cells.size() && i < swaths_by_cells.size();
       ++i) {
    if (swaths_by_cells.at(i).size() == 0) {
      continue;
    }
    F2CCells cell(swath_cells.GetGeometry(i));
    F2CSwathsByCells cell_swaths;
    cell_swaths.emplace_back(swaths_by_cells.at(i));
    f2c::rp::RoutePlannerBase rp;
    cell_routes[i] =
        rp.GenRoute(cell, cell_swaths, false, d_tol, redirect_swaths,
                    time_limit, search_for_optimum);
  }

  return StitchCellRoutes(travel_cells, cell_routes, start_end, d_tol, logger_);
}

}  // namespace autonomy::task::coverage
