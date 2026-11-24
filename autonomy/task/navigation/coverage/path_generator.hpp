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
 * @file path_generator.hpp
 * @brief PathGenerator stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_GENERATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_GENERATOR_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::PathGenerator
 * @brief Turns an ordered route, or a boundary ring, into a driveable path.
 *
 * Every returned state is collision checked against the requested field, so a
 * consumer can send the path to a follower without re-running geometry checks.
 * Corners are rounded with the turning solver chosen in the request. Thin wall
 * bands fall back to a pivot turn instead of producing an infeasible arc.
 */
class PathGenerator {
 public:
  /**
   * @brief Stores the configuration used for every generated path.
   * @param[in] config Coverage configuration (speeds, spacing, robot limits).
   */
  explicit PathGenerator(Configuration config) : config_(std::move(config)) {}

  /**
   * @brief Creates the turning solver selected by the request.
   *
   * The solver type and its continuity mode come from the request; when the
   * mode is left empty the configured defaults are used. The sample distance is
   * applied to the solver so exported paths keep a uniform vertex spacing.
   *
   * @param[in] mode Path mode carrying type, continuity and sample distance.
   * @return Owning pointer to the configured turning solver.
   * @throws CoverageException when the mode or continuity is unknown.
   */
  std::unique_ptr<f2c::pp::TurningBase> CreateCurve(
      const proto::PathMode& mode) const;

  /**
   * @brief Produces a boundary-checked path from the ordered route.
   * @param[in] route Ordered swaths with their connection policy.
   * @param[in] mode Path mode selecting the turning solver.
   * @return Path from the first swath start to the last swath end.
   */
  F2CPath GeneratePath(const F2CRoute& route, const proto::PathMode& mode) const;

  /**
   * @brief Builds driveable, checked perimeter passes.
   *
   * Short edges are simplified first, then every kept edge gets a proportional
   * turn trim so a narrow doorway cannot erase the rest of the ring. Holes are
   * walked directly when a steered connection would leave the field.
   *
   * @param[in] rings Boundary cells whose rings become perimeter passes.
   * @param[in] field Field used for the collision check.
   * @param[in] near Point used to choose the ring start (usually the sweep).
   * @param[in] mode Path mode selecting the turning solver.
   * @return Perimeter path; empty when no ring survived simplification.
   */
  F2CPath GeneratePerimeter(const F2CCells& rings, const F2CCells& field,
                            const F2CPoint& near,
                            const proto::PathMode& mode) const;

 private:
  // Configuration snapshot used for every generated path.
  Configuration config_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_GENERATOR_HPP_
