// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file decomp_generator.hpp
 * @brief DecompGenerator stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief DecompGenerator single coverage-stage responsibility. */
class DecompGenerator {
 public:
  explicit DecompGenerator(Configuration config) : config_(std::move(config)) {}
  /** @brief Split the field using the selected decomposition mode. */
  F2CCells Decompose(const F2CCells&, const proto::DecompMode&) const;

 private:
  Configuration config_;
};
}  // namespace autonomy::task::coverage
