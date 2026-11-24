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
 * @file swath_order_method.hpp
 * @brief SwathOrderMethod route strategy.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_ORDER_METHOD_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_ORDER_METHOD_HPP_

#include "autonomy/task/navigation/coverage/route_method.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief SwathOrderMethod adapted from Open Navigation coverage.
 */
class SwathOrderMethod : public RouteMethod {
 public:
  SwathOrderMethod(const int& logger, RouteType type,
                   std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer)
      : logger_(logger), type_(type), orderer_(std::move(orderer)) {}

  /**
   * @brief Return the complete computed path or plan the requested route.
   */
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

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_ORDER_METHOD_HPP_
