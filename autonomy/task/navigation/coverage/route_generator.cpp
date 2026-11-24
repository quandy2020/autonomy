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
 * @file route_generator.cpp
 * @brief Native complete-coverage route generator implementation.
 */

#include "autonomy/task/navigation/coverage/route_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
#include "autonomy/task/navigation/coverage/swath_order_method.hpp"
#include "autonomy/task/navigation/coverage/tsp_route_method.hpp"
namespace autonomy::task::coverage {
proto::RouteMode RouteGenerator::Resolve(const proto::RouteMode& m) const {
  auto result = m;
  if (!Unknown(m.mode())) {
    if (!result.has_spiral_n()) result.set_spiral_n(4);
    if (!result.has_tsp_d_tol()) result.set_tsp_d_tol(1e-4);
    if (!result.has_tsp_time_limit()) result.set_tsp_time_limit(1);
    if (!result.has_tsp_redirect_swaths()) result.set_tsp_redirect_swaths(true);
    return result;
  }
  result.set_mode(config_.route_type);
  result.set_spiral_n(config_.spiral_n);
  for (auto id : config_.custom_order) result.add_custom_order(id);
  result.set_tsp_redirect_swaths(config_.tsp_redirect_swaths);
  result.set_tsp_time_limit(config_.tsp_time_limit);
  result.set_tsp_d_tol(config_.tsp_d_tol);
  return result;
}
F2CRoute RouteGenerator::GenerateRoute(
    const F2CCells& travel, const F2CCells& cells,
    const F2CSwathsByCells& swaths, const proto::RouteMode& request,
    const std::optional<F2CPoint>& start) const {
  const auto settings = Resolve(request);
  const auto type = RouteTypeFromString(settings.mode());
  const double tol = settings.has_tsp_d_tol() ? settings.tsp_d_tol() : 1e-4;
  Positive(tol, "route tolerance");
  if (settings.has_tsp_time_limit() && settings.tsp_time_limit() > 30)
    throw RequestException("Route optimization time exceeds 30 seconds");
  std::shared_ptr<f2c::rp::SingleCellSwathsOrderBase> orderer;
  switch (type) {
    case RouteType::kBoustrophedon:
      orderer = std::make_shared<f2c::rp::BoustrophedonOrder>();
      break;
    case RouteType::kSnake:
      orderer = std::make_shared<f2c::rp::SnakeOrder>();
      break;
    case RouteType::kSpiral:
      if (!settings.has_spiral_n() || settings.spiral_n() < 2)
        throw CoverageException("SPIRAL requires spiral_n >= 2");
      orderer = std::make_shared<f2c::rp::SpiralOrder>();
      break;
    case RouteType::kCustom: {
      if (cells.size() != 1 ||
          static_cast<size_t>(settings.custom_order_size()) !=
              swaths.SizeTotal())
        throw CoverageException(
            "CUSTOM requires a full permutation for one cell");
      std::set<uint32_t> ids(settings.custom_order().begin(),
                             settings.custom_order().end());
      if (ids.size() != swaths.SizeTotal() || *ids.begin() != 0 ||
          *ids.rbegin() != swaths.SizeTotal() - 1)
        throw CoverageException("Invalid custom permutation");
      orderer = std::make_shared<f2c::rp::CustomOrder>();
      break;
    }
    case RouteType::kTsp: {
      TspRouteMethod method(0, 300);
      return method.Plan(travel, cells, swaths, settings, start);
    }
    default:
      throw CoverageException(
          "Route mode must be BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM or TSP");
  }
  SwathOrderMethod method(0, type, std::move(orderer));
  return method.Plan(travel, cells, swaths, settings, start);
}

}  // namespace autonomy::task::coverage
