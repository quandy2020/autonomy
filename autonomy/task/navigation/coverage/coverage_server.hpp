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
 * @file coverage_server.hpp
 * @brief Native complete-coverage coverage server implementation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_SERVER_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_SERVER_HPP_

#include "autonomy/task/navigation/coverage/decomp_generator.hpp"
#include "autonomy/task/navigation/coverage/headland_generator.hpp"
#include "autonomy/task/navigation/coverage/path_generator.hpp"
#include "autonomy/task/navigation/coverage/robot_params.hpp"
#include "autonomy/task/navigation/coverage/route_generator.hpp"
#include "autonomy/task/navigation/coverage/row_swath_generator.hpp"
#include "autonomy/task/navigation/coverage/swath_generator.hpp"
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::CoverageServer
 * @brief Stateless planner behind the ComputeCoveragePath action.
 *
 * A request either covers one field, or - when an occupancy map is attached -
 * splits the map into rooms, plans every room on its own and stitches the tour
 * through A* links. The server holds configuration only; every goal carries its
 * own geometry and robot limits.
 */
class CoverageServer {
 public:
  /**
   * @brief Stores the configuration used when a request leaves values empty.
   * @param[in] configuration Default coverage configuration.
   */
  explicit CoverageServer(Configuration configuration = {});

  /**
   * @brief Executes the requested coverage planning pipeline.
   *
   * The pipeline runs field validation, headland inset, swath generation,
   * routing, path generation and - for map requests - per-room planning plus
   * the inter-room A* connections. Progress callbacks report the current stage
   * so an action server can publish feedback, and the cancel checker is polled
   * between stages.
   *
   * @param[in] request Goal describing the field, robot limits and modes.
   * @param[in] cancel Optional predicate polled between stages.
   * @param[in] progress Optional callback (@c stage, @c fraction in 0..1).
   * @return Planned path, regions and coverage statistics; the error code is
   *         @c NONE on success and describes the failure otherwise.
   */
  ComputeAction::Result Compute(const proto::CoverageRequest& request,
                                const CancelChecker& cancel = {},
                                const Progress& progress = {}) const;

  /**
   * @brief Accesses the configuration snapshot used by this instance.
   * @return Const reference to the stored configuration.
   */
  const Configuration& Config() const { return config_; }

 private:
  // Configuration snapshot used when a request leaves values empty.
  Configuration config_;
};

/**
 * @brief Exports every path state as sampled poses with its direction.
 *
 * Conversion preserves the metric segment length, the vehicle heading, the
 * travel direction and the work semantics (swath versus turn), so a consumer
 * can rebuild the original path.
 *
 * @param[in] path Path to export.
 * @param[in] field Field providing the output reference point.
 * @param[in] frame Frame id written into every message header.
 * @param[in] cartesian Whether the output stays in the Cartesian frame.
 * @param[in] step Sample distance along each state, metres.
 * @param[out] output Result receiving poses, states and turn metadata.
 */
void ExportPath(const F2CPath& path, const F2CField& field,
                const std::string& frame, bool cartesian, double step,
                ComputeAction::Result& output);

/**
 * @brief Exports the coverage swaths in the requested coordinate frame.
 * @param[in] swaths Swaths to export.
 * @param[in] field Field providing the output reference point.
 * @param[in] frame Frame id written into every message header.
 * @param[in] cartesian Whether the output stays in the Cartesian frame.
 * @param[in] ordered Whether @p swaths already follow the route order.
 * @param[out] output Path components receiving the swath geometry.
 */
void ExportSwaths(const F2CSwaths& swaths, const F2CField& field,
                  const std::string& frame, bool cartesian, bool ordered,
                  proto::PathComponents& output);

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_SERVER_HPP_
