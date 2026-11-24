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
 * @file swath_generator.hpp
 * @brief SwathGenerator stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_GENERATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_GENERATOR_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::SwathGenerator
 * @brief Turns a coverage field into clipped parallel passes.
 *
 * Passes are generated per cell and clipped to the cell outline, therefore a
 * concave field or a field with holes never receives a pass that leaves its
 * traversable area. Spacing comes from the request when present, otherwise from
 * the configured operation width.
 */
class SwathGenerator {
 public:
  /**
   * @brief Stores the configuration used for every generation call.
   * @param[in] config Coverage configuration (spacing, angle, speeds).
   */
  explicit SwathGenerator(Configuration config) : config_(std::move(config)) {}

  /**
   * @brief Generates clipped swaths independently for each cell.
   * @param[in] cells Field, or decomposed blocks, to sweep.
   * @param[in] mode Swath mode carrying the strategy and optional spacing.
   * @return One swath group per input cell, in the same order.
   * @throws CoverageException when the mode is unknown.
   */
  F2CSwathsByCells GenerateSwathsByCells(const F2CCells& cells,
                                         const proto::SwathMode& mode) const;

 private:
  // Configuration snapshot used for every generation call.
  Configuration config_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_SWATH_GENERATOR_HPP_
