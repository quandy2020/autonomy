/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sando_defaults.cpp
 * @brief Ground-robot default values for unset SandoControllerOptions fields.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/sando_defaults.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

void SandoDefaults::Apply(proto::SandoControllerOptions* options) const {
  if (options->motion_model().empty()) {
    options->set_motion_model("diff_drive");
  }
  if (options->horizon() <= 0.0) {
    options->set_horizon(options->lookahead_dist() > 1.0 ? options->lookahead_dist() : 8.0);
  }
  if (options->max_linear_vel() <= 0.0) {
    options->set_max_linear_vel(0.8);
  }
  if (options->max_lateral_vel() <= 0.0) {
    options->set_max_lateral_vel(0.5);
  }
  if (options->max_angular_vel() <= 0.0) {
    options->set_max_angular_vel(1.0);
  }
  if (options->max_linear_accel() <= 0.0) {
    options->set_max_linear_accel(1.5);
  }
  if (options->max_angular_accel() <= 0.0) {
    options->set_max_angular_accel(1.5);
  }
  if (options->j_max() <= 0.0) {
    options->set_j_max(5.0);
  }
  if (options->goal_dist_tol() <= 0.0) {
    options->set_goal_dist_tol(0.25);
  }
  if (options->goal_yaw_tol() <= 0.0) {
    options->set_goal_yaw_tol(0.25);
  }
  if (options->goal_seen_radius() <= 0.0) {
    options->set_goal_seen_radius(std::max(1.5, options->goal_dist_tol() * 4.0));
  }
  if (options->num_segments() <= 0) {
    options->set_num_segments(4);
  }
  if (options->num_polytopes() <= 0) {
    options->set_num_polytopes(3);
  }
  if (options->inflation() <= 0.0) {
    options->set_inflation(0.25);
  }
  if (options->robot_radius() <= 0.0) {
    options->set_robot_radius(0.22);
  }
  if (options->dc() <= 0.0) {
    options->set_dc(0.05);
  }
  if (options->factor_initial() <= 0.0) {
    options->set_factor_initial(1.0);
  }
  if (options->factor_final() <= options->factor_initial()) {
    options->set_factor_final(options->factor_initial() + 1.5);
  }
  if (options->factor_step() <= 0.0) {
    options->set_factor_step(0.1);
  }
  if (options->jerk_weight() <= 0.0) {
    options->set_jerk_weight(10.0);
  }
  if (options->w_max_yawing() <= 0.0) {
    options->set_w_max_yawing(0.6);
  }
  if (options->alpha_filter_dyaw() <= 0.0) {
    options->set_alpha_filter_dyaw(0.8);
  }
  if (options->default_k() <= 0) {
    options->set_default_k(50);
  }
  if (options->k_value_factor() <= 0.0) {
    options->set_k_value_factor(5.0);
  }
  if (options->heuristic_weight() <= 0.0) {
    options->set_heuristic_weight(2.0);
  }
  if (options->heat_weight() <= 0.0) {
    options->set_heat_weight(10.0);
  }
  if (options->static_heat_rmax() <= 0.0) {
    options->set_static_heat_rmax(3.0 * options->robot_radius());
  }
  if (options->static_heat_alpha() <= 0.0) {
    options->set_static_heat_alpha(5.0);
  }
  if (options->obst_max_vel() <= 0.0) {
    options->set_obst_max_vel(0.8);
  }
  if (options->traj_lifetime() <= 0.0) {
    options->set_traj_lifetime(2.0);
  }
  if (options->prediction_horizon() <= 0.0) {
    options->set_prediction_horizon(1.0);
  }
  if (options->max_dist_vertexes() <= 0.0) {
    options->set_max_dist_vertexes(1.0);
  }
  if (options->sfc_bbox() <= 0.0) {
    options->set_sfc_bbox(3.0);
  }
  if (options->hover_d_trigger() <= 0.0) {
    options->set_hover_d_trigger(1.5);
  }
  if (options->hover_evasion() <= 0.0) {
    options->set_hover_evasion(1.0);
  }
  if (options->max_expansions() <= 0) {
    options->set_max_expansions(80000);
  }
  if (options->global_planner().empty()) {
    options->set_global_planner("astar_heat");
  }
  if (options->decay_len_cells() <= 0.0) {
    options->set_decay_len_cells(100.0);
  }
  if (options->min_len() <= 0.0) {
    options->set_min_len(0.5);
  }
  if (options->map_buffer() <= 0.0) {
    options->set_map_buffer(4.0);
  }
  if (options->hgp_timeout_ms() <= 0) {
    options->set_hgp_timeout_ms(300);
  }
  if (options->dynamic_constraint().empty()) {
    options->set_dynamic_constraint("Linf");
  }
  if (options->yaw_spinning_threshold() <= 0) {
    options->set_yaw_spinning_threshold(10000);
  }
  if (options->yaw_spinning_dyaw() <= 0.0) {
    options->set_yaw_spinning_dyaw(0.8);
  }
  if (options->min_cluster_cells() <= 0) {
    options->set_min_cluster_cells(3);
  }
  if (options->max_cluster_cells() <= options->min_cluster_cells()) {
    options->set_max_cluster_cells(400);
  }
  if (options->accel_threshold() <= 0.0) {
    options->set_accel_threshold(1.5);
  }
  if (options->kf_alpha() <= 0.0 || options->kf_alpha() >= 1.0) {
    options->set_kf_alpha(0.9);
  }
  if (options->velocity_threshold() <= 0.0) {
    options->set_velocity_threshold(0.2);
  }
  if (options->environment_assumption().empty()) {
    options->set_environment_assumption("dynamic");
  }
  if (options->hover_lookahead() <= 0.0) {
    options->set_hover_lookahead(15.0);
  }
  if (!options->has_dynamic_as_occupied_current()) {
    options->set_dynamic_as_occupied_current(true);
  }
  if (!options->has_dynamic_as_occupied_future()) {
    options->set_dynamic_as_occupied_future(false);
  }
  if (!options->has_stop_when_occupied()) {
    options->set_stop_when_occupied(true);
  }
  if (!options->has_dynamic_heat()) {
    options->set_dynamic_heat(true);
  }
  if (!options->has_hover_avoidance()) {
    options->set_hover_avoidance(false);
  }
  if (!options->has_skip_initial_yawing()) {
    options->set_skip_initial_yawing(false);
  }
  if (!options->has_inflate_unknown()) {
    options->set_inflate_unknown(true);
  }
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
