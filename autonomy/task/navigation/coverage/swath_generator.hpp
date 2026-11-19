// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file swath_generator.hpp
 * @brief SwathGenerator stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief SwathGenerator single coverage-stage responsibility. */
class SwathGenerator {
 public:
  explicit SwathGenerator(Configuration config) : config_(std::move(config)) {}
  /** @brief Generate clipped swaths independently for each coverage cell. */
  F2CSwathsByCells GenerateSwathsByCells(const F2CCells&,
                                         const proto::SwathMode&) const;

 private:
  Configuration config_;
};
}  // namespace autonomy::task::coverage
