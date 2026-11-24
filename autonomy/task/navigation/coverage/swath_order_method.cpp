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
 * @file swath_order_method.cpp
 * @brief Native complete-coverage swath order method implementation.
 */

#include "autonomy/task/navigation/coverage/swath_order_method.hpp"

#include "autonomy/task/navigation/coverage/route_geometry.hpp"
namespace autonomy::task::coverage {
F2CRoute SwathOrderMethod::Plan(const F2CCells &travel_cells,
                                const F2CCells &swath_cells,
                                const F2CSwathsByCells &swaths_by_cells,
                                const proto::RouteMode &settings,
                                const std::optional<F2CPoint> &start_end) {
  // CUSTOM indexes swaths absolutely and validates the vector length, so one
  // order vector cannot be split across cells.
  if (type_ == RouteType::kCustom && swath_cells.size() > 1) {
    throw CoverageException(
        "CUSTOM route mode is not supported with field decomposition; the "
        "order "
        "vector cannot be split across cells. Use another route mode or "
        "disable it.");
  }

  if (type_ == RouteType::kSpiral) {
    dynamic_cast<f2c::rp::SpiralOrder *>(orderer_.get())
        ->SetSpiralSize(settings.spiral_n());
  } else if (type_ == RouteType::kCustom) {
    std::vector<size_t> custom_order(settings.custom_order().begin(),
                                     settings.custom_order().end());
    dynamic_cast<f2c::rp::CustomOrder *>(orderer_.get())
        ->SetCustomOrder(custom_order);
  }

  // Single cell: unchanged behaviour, one genSortedSwaths call over the
  // flattened swaths.
  if (swath_cells.size() <= 1) {
    F2CSwaths ordered = orderer_->GenSortedSwaths(swaths_by_cells.flatten());
    F2CRoute route;
    route.AddConnectedSwaths(F2CMultiPoint(), ordered);
    return route;
  }

  // Multi-cell: order each cell alone. Flattening first would collide the
  // per-cell swath ids that genSortedSwaths sorts on, interleaving the cells.
  const double d_tol = settings.tsp_d_tol() > 0.0 ? settings.tsp_d_tol() : 1e-4;
  std::vector<F2CRoute> cell_routes(swath_cells.size());
  for (size_t i = 0; i < swath_cells.size() && i < swaths_by_cells.size();
       ++i) {
    if (swaths_by_cells.at(i).size() == 0) {
      continue;
    }
    cell_routes[i] = RouteThroughCell(
        swath_cells.GetGeometry(i),
        orderer_->GenSortedSwaths(swaths_by_cells.at(i)), d_tol, logger_);
  }
  return StitchCellRoutes(travel_cells, cell_routes, start_end, d_tol, logger_);
}

}  // namespace autonomy::task::coverage
