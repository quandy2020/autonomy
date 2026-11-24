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
 * @file headland_generator.hpp
 * @brief HeadlandGenerator stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_HEADLAND_GENERATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_HEADLAND_GENERATOR_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::HeadlandGenerator
 * @brief Builds and removes the headland band around a coverage field.
 *
 * The class keeps only the configuration: the band itself is derived from the
 * request each time, so a caller can plan rooms with different wall bands in
 * one process.
 */
class HeadlandGenerator {
 public:
  /**
   * @brief Stores the configuration used for every band operation.
   * @param[in] config Coverage configuration (band width, spacing, robot size).
   */
  explicit HeadlandGenerator(Configuration config)
      : config_(std::move(config)) {}

  /**
   * @brief Resolves the requested headland width in metres.
   * @param[in] mode Headland mode; only CONSTANT is supported today.
   * @return Band width in metres, taken from the request or the configuration.
   * @throws CoverageException when the mode is unsupported.
   * @throws RequestException when the width is not positive.
   */
  double Width(const proto::HeadlandMode& mode) const;

  /**
   * @brief Shrinks the field by the requested band.
   *
   * The returned cells keep only valid polygon components; the removed strip is
   * the area the perimeter pass later covers.
   *
   * @param[in] cells Field to inset.
   * @param[in] mode Headland mode carrying the band width.
   * @return Inset field (the core area).
   * @throws RequestException when the band removes the whole field.
   */
  F2CCells GenerateHeadlands(const F2CCells& cells,
                             const proto::HeadlandMode& mode) const;

  /**
   * @brief Carves corridors only along borders shared by two cells.
   *
   * Used when a decomposed field must keep a headland between neighbouring
   * blocks; internal borders are widened by @p width, outer borders are left
   * untouched.
   *
   * @param[in] cells Decomposed blocks.
   * @param[in] width Corridor width in metres.
   * @return Cells with the shared-border corridors removed.
   * @throws RequestException when no swath cell survives the carve.
   */
  F2CCells GenerateHeadlandsBetweenCells(const F2CCells& cells,
                                         double width) const;

  /**
   * @brief Generates the concentric passes of the headland band.
   *
   * @param[in] field Field whose boundary is swept.
   * @param[in] width Center-to-center distance between two passes.
   * @param[in] mode Headland mode describing the band.
   * @param[in] passes Explicit ring count; a non-positive value falls back to
   *             round(band width / pass spacing), the historical behaviour.
   * @return One cell group per ring, outermost first.
   * @throws RequestException when @p width is not positive or @p passes is huge.
   */
  std::vector<F2CCells> GenerateHeadlandSwaths(
      const F2CCells& field, double width, const proto::HeadlandMode& mode,
      int passes = -1) const;

 private:
  // Configuration snapshot used for every band operation.
  Configuration config_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_HEADLAND_GENERATOR_HPP_
