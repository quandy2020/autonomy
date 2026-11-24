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
 * @file decomp_generator.hpp
 * @brief DecompGenerator stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_DECOMP_GENERATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_DECOMP_GENERATOR_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::DecompGenerator
 * @brief Splits a concave field into blocks that are simple to sweep.
 *
 * A single serpentine over a concave outline leaves long joins through the
 * middle of the field; decomposing first confines those joins to the shared
 * borders of the blocks.
 */
class DecompGenerator {
 public:
  /**
   * @brief Stores the configuration used for every decomposition.
   * @param[in] config Coverage configuration (split angle, route defaults).
   */
  explicit DecompGenerator(Configuration config) : config_(std::move(config)) {}

  /**
   * @brief Splits the field using the selected decomposition mode.
   * @param[in] cells Field to decompose.
   * @param[in] mode Decomposition mode and its optional parameters.
   * @return Non-empty blocks whose union equals @p cells.
   * @throws CoverageException when the mode is unknown.
   */
  F2CCells Decompose(const F2CCells& cells,
                     const proto::DecompMode& mode) const;

 private:
  // Configuration snapshot used for every decomposition.
  Configuration config_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_DECOMP_GENERATOR_HPP_
