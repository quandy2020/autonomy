// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file configuration.hpp
 * @brief Defaults for coverage algorithms and native execution. */
#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace autonomy::task::coverage {
/** @brief Immutable per-runtime coverage configuration snapshot. */
struct Configuration {
  bool cartesian{true};
  double robot_width{1.5}, operation_width{3.0}, min_turning_radius{1.0};
  /** Cleaning radius in metres; the swath width is twice this value.
   *  Zero keeps the explicit operation_width. */
  double robot_radius{0.0};
  double linear_curv_change{0.3}, cruise_speed{1.0}, turn_speed{0.5};
  bool allow_overlap{false};
  double headland_width{3.0};
  std::string headland_type{"CONSTANT"}, swath_type{"NUMBER_MODIFIED"},
      swath_angle_type{"BRUTE_FORCE"};
  double swath_angle{0.0}, step_angle{0.0174532925199433};
  std::string route_type{"BOUSTROPHEDON"};
  unsigned spiral_n{4};
  std::vector<uint32_t> custom_order;
  bool tsp_redirect_swaths{true};
  unsigned tsp_time_limit{1};
  double tsp_d_tol{1e-4};
  std::string path_type{"DUBIN"}, path_continuity_type{"DISCONTINUOUS"};
  double turn_point_distance{0.05};
  std::string decomp_type{"BOUSTROPHEDON"};
  double decomp_split_angle{0.0};
  double navigation_timeout{600.0};
  std::size_t max_transport_poses{40};
  bool controller_supports_reverse{false};
  std::string planner_id{"navfn_planner"};  ///< Native planner used to reach the coverage start.
  std::string controller_id{"rpp_controller"};
};
}  // namespace autonomy::task::coverage
