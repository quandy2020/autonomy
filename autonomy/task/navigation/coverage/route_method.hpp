// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file route_method.hpp
 * @brief RouteMethod route strategy. */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief RouteMethod adapted from Open Navigation coverage. */
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
