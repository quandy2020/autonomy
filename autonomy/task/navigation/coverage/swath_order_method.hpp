// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file swath_order_method.hpp
 * @brief SwathOrderMethod route strategy. */
#pragma once
#include "autonomy/task/navigation/coverage/route_method.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief SwathOrderMethod adapted from Open Navigation coverage. */
class SwathOrderMethod : public RouteMethod {
 public:
  SwathOrderMethod(const int& logger, RouteType type,
                   std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer)
      : logger_(logger), type_(type), orderer_(std::move(orderer)) {}

  /** @brief Return the complete computed path or plan the requested route. */
  F2CRoute Plan(
      const F2CCells& travel_cells, const F2CCells& swath_cells,
      const F2CSwathsByCells& swaths_by_cells, const proto::RouteMode& settings,
      const std::optional<F2CPoint>& start_end_point = std::nullopt) override;

 private:
  int logger_;
  RouteType type_;
  std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer_;
};
}  // namespace autonomy::task::coverage
