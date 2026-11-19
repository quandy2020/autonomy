// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file path_generator.hpp
 * @brief PathGenerator stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief PathGenerator single coverage-stage responsibility. */
class PathGenerator {
 public:
  explicit PathGenerator(Configuration config) : config_(std::move(config)) {}
  /** @brief Create a selected turning solver with the requested continuity and
   * sample distance. */
  std::unique_ptr<f2c::pp::TurningBase> CreateCurve(
      const proto::PathMode&) const;
  /** @brief Produce a boundary-checked path from the ordered route. */
  F2CPath GeneratePath(const F2CRoute&, const proto::PathMode&) const;
  /** @brief Build driveable, checked perimeter passes. */
  F2CPath GeneratePerimeter(const F2CCells&, const F2CCells&, const F2CPoint&,
                            const proto::PathMode&) const;

 private:
  Configuration config_;
};
}  // namespace autonomy::task::coverage
