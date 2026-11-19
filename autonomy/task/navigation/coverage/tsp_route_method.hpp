// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file tsp_route_method.hpp
 * @brief TspRouteMethod route strategy. */
#pragma once
#include "autonomy/task/navigation/coverage/route_method.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief TspRouteMethod adapted from Open Navigation coverage. */
class TspRouteMethod : public RouteMethod {
 public:
  // max_swaths_for_global_route: unused, kept for ABI/call-site stability
  TspRouteMethod(const int& logger, size_t max_swaths_for_global_route)
      : logger_(logger),
        max_swaths_for_global_route_(max_swaths_for_global_route) {}

  /** @brief Return the complete computed path or plan the requested route. */
  F2CRoute Plan(
      const F2CCells& travel_cells, const F2CCells& swath_cells,
      const F2CSwathsByCells& swaths_by_cells, const proto::RouteMode& settings,
      const std::optional<F2CPoint>& start_end_point = std::nullopt) override;

 private:
  int logger_;
  size_t max_swaths_for_global_route_;
};
}  // namespace autonomy::task::coverage
