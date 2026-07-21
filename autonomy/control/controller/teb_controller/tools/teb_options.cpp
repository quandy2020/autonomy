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

#include <functional>

#include "autonomy/control/controller/teb_controller/tools/teb_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {
namespace {

void SetDoubleIfPresent(
    autonomy::common::LuaParameterDictionary* dict, const std::string& key,
    const std::function<void(double)>& setter) {
  if (dict->HasKey(key)) {
    setter(dict->GetDouble(key));
  }
}

void SetIntIfPresent(
    autonomy::common::LuaParameterDictionary* dict, const std::string& key,
    const std::function<void(int)>& setter) {
  if (dict->HasKey(key)) {
    setter(dict->GetInt(key));
  }
}

void SetBoolIfPresent(
    autonomy::common::LuaParameterDictionary* dict, const std::string& key,
    const std::function<void(bool)>& setter) {
  if (dict->HasKey(key)) {
    setter(dict->GetBool(key));
  }
}

void SetStringIfPresent(
    autonomy::common::LuaParameterDictionary* dict, const std::string& key,
    const std::function<void(const std::string&)>& setter) {
  if (dict->HasKey(key)) {
    setter(dict->GetString(key));
  }
}

}  // namespace

proto::TEBControllerOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* parameter_dictionary) {
  proto::TEBControllerOptions options;
  if (!parameter_dictionary) {
    return options;
  }

  SetDoubleIfPresent(parameter_dictionary, "max_vel_x",
                     [&](double v) { options.set_max_vel_x(v); });
  SetDoubleIfPresent(parameter_dictionary, "max_vel_x_backwards",
                     [&](double v) { options.set_max_vel_x_backwards(v); });
  SetDoubleIfPresent(parameter_dictionary, "max_vel_theta",
                     [&](double v) { options.set_max_vel_theta(v); });
  SetDoubleIfPresent(parameter_dictionary, "acc_lim_x",
                     [&](double v) { options.set_acc_lim_x(v); });
  SetDoubleIfPresent(parameter_dictionary, "acc_lim_theta",
                     [&](double v) { options.set_acc_lim_theta(v); });
  SetDoubleIfPresent(parameter_dictionary, "min_turning_radius",
                     [&](double v) { options.set_min_turning_radius(v); });
  SetStringIfPresent(parameter_dictionary, "robot_model",
                     [&](const std::string& v) { options.set_robot_model(v); });

  SetDoubleIfPresent(parameter_dictionary, "dt_ref",
                     [&](double v) { options.set_dt_ref(v); });
  SetDoubleIfPresent(parameter_dictionary, "dt_hysteresis",
                     [&](double v) { options.set_dt_hysteresis(v); });
  SetIntIfPresent(parameter_dictionary, "min_samples",
                  [&](int v) { options.set_min_samples(v); });
  SetIntIfPresent(parameter_dictionary, "max_samples",
                  [&](int v) { options.set_max_samples(v); });
  SetDoubleIfPresent(parameter_dictionary, "max_global_plan_lookahead_dist",
                     [&](double v) {
                       options.set_max_global_plan_lookahead_dist(v);
                     });
  SetDoubleIfPresent(parameter_dictionary, "global_plan_prune_distance",
                     [&](double v) { options.set_global_plan_prune_distance(v); });
  SetIntIfPresent(parameter_dictionary, "control_look_ahead_poses",
                  [&](int v) { options.set_control_look_ahead_poses(v); });
  SetBoolIfPresent(parameter_dictionary, "global_plan_overwrite_orientation",
                   [&](bool v) {
                     options.set_global_plan_overwrite_orientation(v);
                   });

  SetDoubleIfPresent(parameter_dictionary, "min_obstacle_dist",
                     [&](double v) { options.set_min_obstacle_dist(v); });
  SetDoubleIfPresent(parameter_dictionary, "inflation_dist",
                     [&](double v) { options.set_inflation_dist(v); });
  SetBoolIfPresent(parameter_dictionary, "include_costmap_obstacles",
                   [&](bool v) { options.set_include_costmap_obstacles(v); });
  SetDoubleIfPresent(parameter_dictionary,
                     "costmap_obstacles_behind_robot_dist",
                     [&](double v) {
                       options.set_costmap_obstacles_behind_robot_dist(v);
                     });
  SetIntIfPresent(parameter_dictionary, "obstacle_poses_affected",
                  [&](int v) { options.set_obstacle_poses_affected(v); });
  SetDoubleIfPresent(parameter_dictionary,
                     "costmap_obstacle_sample_resolution",
                     [&](double v) {
                       options.set_costmap_obstacle_sample_resolution(v);
                     });

  SetIntIfPresent(parameter_dictionary, "no_inner_iterations",
                  [&](int v) { options.set_no_inner_iterations(v); });
  SetIntIfPresent(parameter_dictionary, "no_outer_iterations",
                  [&](int v) { options.set_no_outer_iterations(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_max_vel_x",
                     [&](double v) { options.set_weight_max_vel_x(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_max_vel_theta",
                     [&](double v) { options.set_weight_max_vel_theta(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_acc_lim_x",
                     [&](double v) { options.set_weight_acc_lim_x(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_acc_lim_theta",
                     [&](double v) { options.set_weight_acc_lim_theta(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_kinematics_nh",
                     [&](double v) { options.set_weight_kinematics_nh(v); });
  SetDoubleIfPresent(
      parameter_dictionary, "weight_kinematics_forward_drive",
      [&](double v) { options.set_weight_kinematics_forward_drive(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_kinematics_turning_radius",
                     [&](double v) {
                       options.set_weight_kinematics_turning_radius(v);
                     });
  SetDoubleIfPresent(parameter_dictionary, "weight_optimaltime",
                     [&](double v) { options.set_weight_optimaltime(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_obstacle",
                     [&](double v) { options.set_weight_obstacle(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_viapoint",
                     [&](double v) { options.set_weight_viapoint(v); });
  SetDoubleIfPresent(parameter_dictionary, "weight_shortest_path",
                     [&](double v) { options.set_weight_shortest_path(v); });
  SetDoubleIfPresent(parameter_dictionary, "penalty_epsilon",
                     [&](double v) { options.set_penalty_epsilon(v); });

  SetDoubleIfPresent(parameter_dictionary, "transform_tolerance",
                     [&](double v) { options.set_transform_tolerance(v); });
  SetDoubleIfPresent(parameter_dictionary, "max_robot_pose_search_dist",
                     [&](double v) { options.set_max_robot_pose_search_dist(v); });
  SetBoolIfPresent(parameter_dictionary, "free_goal_vel",
                   [&](bool v) { options.set_free_goal_vel(v); });

  return options;
}

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
