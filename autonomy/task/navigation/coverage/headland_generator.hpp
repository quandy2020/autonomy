// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file headland_generator.hpp
 * @brief HeadlandGenerator stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief HeadlandGenerator single coverage-stage responsibility. */
class HeadlandGenerator {
 public:
  explicit HeadlandGenerator(Configuration config)
      : config_(std::move(config)) {}
  /** @brief Resolve the requested headland width in metres. */
  double Width(const proto::HeadlandMode& mode) const;
  /** @brief Remove the requested headland while preserving valid polygon
   * components. */
  F2CCells GenerateHeadlands(const F2CCells&, const proto::HeadlandMode&) const;
  /** @brief Carve corridors only along shared cell borders. */
  F2CCells GenerateHeadlandsBetweenCells(const F2CCells&, double) const;
  /** @brief Generate concentric passes for the headland band. */
  std::vector<F2CCells> GenerateHeadlandSwaths(
      const F2CCells&, double, const proto::HeadlandMode&) const;

 private:
  Configuration config_;
};
}  // namespace autonomy::task::coverage
