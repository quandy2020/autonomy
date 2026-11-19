// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file route_generator.hpp
 * @brief RouteGenerator stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief RouteGenerator single coverage-stage responsibility. */
class RouteGenerator {
 public:
  explicit RouteGenerator(Configuration config) : config_(std::move(config)) {}
  /** @brief Resolve the complete route request against immutable defaults. */
  proto::RouteMode Resolve(const proto::RouteMode&) const;
  /** @brief Order swaths and retain the cell-to-cell connection geometry. */
  F2CRoute GenerateRoute(const F2CCells&, const F2CCells&,
                         const F2CSwathsByCells&, const proto::RouteMode&,
                         const std::optional<F2CPoint>& = {}) const;

 private:
  Configuration config_;
};
}  // namespace autonomy::task::coverage
