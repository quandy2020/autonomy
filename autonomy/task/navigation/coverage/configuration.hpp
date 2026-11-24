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
 * @file configuration.hpp
 * @brief Defaults for coverage algorithms and native execution.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_HPP_

#include <string>
#include <vector>
#include <cstdint>
namespace autonomy::task::coverage {

/**
 * @brief Immutable per-runtime coverage configuration snapshot.
 *
 * Populated from the JSON file passed with @c --coverage_config and then copied
 * into every request, so a single process cannot change the robot limits while
 * a goal is being planned. Distances are metres, speeds are metres per second,
 * angles are radians.
 */
struct Configuration {
  // Interpret request coordinates as a local Cartesian frame.
  bool cartesian{true};

  // Physical robot width, used for the wall margin and the band geometry.
  double robot_width{1.5};

  // Center-to-center distance between two coverage passes.
  double operation_width{3.0};

  // Smallest curvature radius the robot can drive.
  double min_turning_radius{1.0};

  // Robot radius used for map segmentation and clearance checks.
  double robot_radius{0.4};

  // Planner id forwarded to ComputePathToPose for the approach leg.
  std::string planner_id{"navfn_planner"};

  // Allowed change of curvature between consecutive path states.
  double linear_curv_change{0.3};

  // Speed on straight coverage passes.
  double cruise_speed{1.0};

  // Speed while turning or pivoting.
  double turn_speed{0.5};

  // Allow neighbouring passes to overlap slightly.
  bool allow_overlap{false};

  // Default headland band width when the request leaves it empty.
  double headland_width{3.0};

  // Explicit wall ring count; 0 keeps the width/spacing heuristic.
  unsigned headland_passes{1};

  // Headland computation mode (CONSTANT today).
  std::string headland_type{"CONSTANT"};

  // Swath generation mode.
  std::string swath_type{"NUMBER_MODIFIED"};

  // Strategy used to pick the swath angle.
  std::string swath_angle_type{"BRUTE_FORCE"};

  // Fixed swath angle in radians when the angle type needs one.
  double swath_angle{0.0};

  // Angular step of the brute-force angle search, radians.
  double step_angle{0.0174532925199433};

  // Route order used to visit the swaths.
  std::string route_type{"BOUSTROPHEDON"};

  // Spiral turn count for the SPIRAL route.
  unsigned spiral_n{4};

  // Explicit swath order, when the request provides one.
  std::vector<uint32_t> custom_order;

  // Allow the TSP route to reverse individual swaths.
  bool tsp_redirect_swaths{true};

  // Time budget of the TSP solver, seconds.
  unsigned tsp_time_limit{1};

  // Acceptance tolerance of the TSP solver, metres.
  double tsp_d_tol{1e-4};

  // Turning primitive used to join swaths (DUBIN or REEDS_SHEPP).
  std::string path_type{"DUBIN"};

  // Curvature continuity of the turning primitive.
  std::string path_continuity_type{"DISCONTINUOUS"};

  // Vertex spacing of an exported path, metres.
  double turn_point_distance{0.05};

  // Decomposition strategy used before routing.
  std::string decomp_type{"BOUSTROPHEDON"};

  // Split angle of the decomposition, radians.
  double decomp_split_angle{0.0};

  // Overall navigation timeout of one coverage goal, seconds.
  double navigation_timeout{600.0};

  // Maximum poses sent per follow-path segment.
  std::size_t max_transport_poses{40};

  // Whether the selected controller can drive backwards.
  bool controller_supports_reverse{false};

  // Follow-path controller id used for every segment.
  std::string controller_id{"rpp_controller"};
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_HPP_
