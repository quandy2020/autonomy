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
 * @file route_generator.hpp
 * @brief RouteGenerator stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GENERATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GENERATOR_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::RouteGenerator
 * @brief Orders the generated swaths into a driveable route.
 *
 * The generator keeps the cell-to-cell connection geometry, so the path stage
 * can leave the field through a doorway instead of cutting through a wall. The
 * request mode wins over the configured default; an unknown mode is rejected
 * before any swath is reordered.
 */
class RouteGenerator {
 public:
  /**
   * @brief Stores the configuration used for every route.
   * @param[in] config Coverage configuration (route type, TSP limits, speeds).
   */
  explicit RouteGenerator(Configuration config) : config_(std::move(config)) {}

  /**
   * @brief Resolves a route request against the immutable defaults.
   *
   * Empty fields fall back to the stored configuration, while an unknown mode
   * name is rejected so a typo cannot silently change the tour shape.
   *
   * @param[in] mode Requested route mode, possibly partially filled.
   * @return Fully populated route mode.
   * @throws CoverageException when the mode cannot be resolved.
   */
  proto::RouteMode Resolve(const proto::RouteMode& mode) const;

  /**
   * @brief Orders the swaths and keeps their connection geometry.
   *
   * @param[in] travel Cells the route is allowed to travel through.
   * @param[in] swath_cells Cells the swaths were generated in.
   * @param[in] grouped Swaths grouped per cell, in generation order.
   * @param[in] mode Route mode; resolved with @ref Resolve first.
   * @param[in] start Optional tour start; used by the TSP route.
   * @return Ordered route with its inter-swath connections.
   */
  F2CRoute GenerateRoute(const F2CCells& travel, const F2CCells& swath_cells,
                         const F2CSwathsByCells& grouped,
                         const proto::RouteMode& mode,
                         const std::optional<F2CPoint>& start = {}) const;

 private:
  // Configuration snapshot used for every route.
  Configuration config_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ROUTE_GENERATOR_HPP_
