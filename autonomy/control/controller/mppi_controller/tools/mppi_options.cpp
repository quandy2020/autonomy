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

 #include "autonomy/control/controller/mppi_controller/tools/mppi_options.hpp"

 #include "autonomy/common/lua_parameter_dictionary.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace tools {
 
 proto::MPPIControllerOptions LoadOptions(
     ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
     proto::MPPIControllerOptions options;
 
     if (!parameter_dictionary) {
         return options;
     }
 
     // GoalChecker
     if (parameter_dictionary->HasKey("goal_checker")) {
         auto dict = parameter_dictionary->GetDictionary("goal_checker");
         if (dict) {
             auto* goal_checker = options.mutable_goal_checker();
             if (dict->HasKey("plugin")) {
                 goal_checker->set_plugin(dict->GetString("plugin"));
             }
             if (dict->HasKey("stateful")) {
                 goal_checker->set_stateful(dict->GetBool("stateful"));
             }
             if (dict->HasKey("xy_goal_tolerance")) {
                 goal_checker->set_xy_goal_tolerance(
                     dict->GetDouble("xy_goal_tolerance"));
             }
             if (dict->HasKey("yaw_goal_tolerance")) {
                 goal_checker->set_yaw_goal_tolerance(
                     dict->GetDouble("yaw_goal_tolerance"));
             }
             if (dict->HasKey("path_length_tolerance")) {
                 goal_checker->set_path_length_tolerance(
                     dict->GetDouble("path_length_tolerance"));
             }
         }
     }
 
     // ProgressChecker
     if (parameter_dictionary->HasKey("progress_checker")) {
         auto dict = parameter_dictionary->GetDictionary("progress_checker");
         if (dict) {
             auto* progress_checker = options.mutable_progress_checker();
             if (dict->HasKey("plugin")) {
                 progress_checker->set_plugin(dict->GetString("plugin"));
             }
             if (dict->HasKey("required_movement_radius")) {
                 progress_checker->set_required_movement_radius(
                     dict->GetDouble("required_movement_radius"));
             }
             if (dict->HasKey("movement_time_allowance")) {
                 progress_checker->set_movement_time_allowance(
                     dict->GetDouble("movement_time_allowance"));
             }
         }
     }
 
     // Core Optimization Parameters
     if (parameter_dictionary->HasKey("time_steps")) {
         options.set_time_steps(parameter_dictionary->GetInt("time_steps"));
     }
     if (parameter_dictionary->HasKey("model_dt")) {
         options.set_model_dt(parameter_dictionary->GetDouble("model_dt"));
     }
     if (parameter_dictionary->HasKey("batch_size")) {
         options.set_batch_size(parameter_dictionary->GetInt("batch_size"));
     }
 
     // Noise Parameters
     if (parameter_dictionary->HasKey("vx_std")) {
         options.set_vx_std(parameter_dictionary->GetDouble("vx_std"));
     }
     if (parameter_dictionary->HasKey("vy_std")) {
         options.set_vy_std(parameter_dictionary->GetDouble("vy_std"));
     }
     if (parameter_dictionary->HasKey("wz_std")) {
         options.set_wz_std(parameter_dictionary->GetDouble("wz_std"));
     }
 
     // Velocity Constraints
     if (parameter_dictionary->HasKey("vx_max")) {
         options.set_vx_max(parameter_dictionary->GetDouble("vx_max"));
     }
     if (parameter_dictionary->HasKey("vx_min")) {
         options.set_vx_min(parameter_dictionary->GetDouble("vx_min"));
     }
     if (parameter_dictionary->HasKey("vy_max")) {
         options.set_vy_max(parameter_dictionary->GetDouble("vy_max"));
     }
     if (parameter_dictionary->HasKey("wz_max")) {
         options.set_wz_max(parameter_dictionary->GetDouble("wz_max"));
     }
 
     // Optimization Algorithm Parameters
     if (parameter_dictionary->HasKey("iteration_count")) {
         options.set_iteration_count(
             parameter_dictionary->GetInt("iteration_count"));
     }
     if (parameter_dictionary->HasKey("temperature")) {
         options.set_temperature(parameter_dictionary->GetDouble("temperature"));
     }
     if (parameter_dictionary->HasKey("gamma")) {
         options.set_gamma(parameter_dictionary->GetDouble("gamma"));
     }
 
     // Motion Model
     if (parameter_dictionary->HasKey("motion_model")) {
         options.set_motion_model(
             parameter_dictionary->GetString("motion_model"));
     }
 
     // Visualization
     if (parameter_dictionary->HasKey("visualize")) {
         options.set_visualize(parameter_dictionary->GetBool("visualize"));
     }
 
     // TrajectoryVisualizer
     if (parameter_dictionary->HasKey("TrajectoryVisualizer")) {
         auto tv_dict =
             parameter_dictionary->GetDictionary("TrajectoryVisualizer");
         if (tv_dict) {
             auto* tv = options.mutable_trajectory_visualizer();
             if (tv_dict->HasKey("trajectory_step")) {
                 tv->set_trajectory_step(tv_dict->GetInt("trajectory_step"));
             }
             if (tv_dict->HasKey("time_step")) {
                 tv->set_time_step(tv_dict->GetInt("time_step"));
             }
         }
     }
 
     // AckermannConstraints
     if (parameter_dictionary->HasKey("AckermannConstraints")) {
         auto ack_dict =
             parameter_dictionary->GetDictionary("AckermannConstraints");
         if (ack_dict) {
             auto* ack = options.mutable_ackermann_constraints();
             if (ack_dict->HasKey("min_turning_r")) {
                 ack->set_min_turning_r(ack_dict->GetDouble("min_turning_r"));
             }
         }
     }
 
     // Critics list
     if (parameter_dictionary->HasKey("critics")) {
         auto critics = parameter_dictionary->GetArrayValuesAsStrings();
         for (const auto& critic : critics) {
             options.add_critics(critic);
         }
     }
 
     // ConstraintCritic
     if (parameter_dictionary->HasKey("ConstraintCritic")) {
         auto dict = parameter_dictionary->GetDictionary("ConstraintCritic");
         if (dict) {
             auto* critic = options.mutable_constraint_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
         }
     }
 
     // GoalCritic
     if (parameter_dictionary->HasKey("GoalCritic")) {
         auto dict = parameter_dictionary->GetDictionary("GoalCritic");
         if (dict) {
             auto* critic = options.mutable_goal_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
         }
     }
 
     // GoalAngleCritic
     if (parameter_dictionary->HasKey("GoalAngleCritic")) {
         auto dict = parameter_dictionary->GetDictionary("GoalAngleCritic");
         if (dict) {
             auto* critic = options.mutable_goal_angle_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
         }
     }
 
     // PreferForwardCritic
     if (parameter_dictionary->HasKey("PreferForwardCritic")) {
         auto dict = parameter_dictionary->GetDictionary("PreferForwardCritic");
         if (dict) {
             auto* critic = options.mutable_prefer_forward_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
         }
     }
 
     // CostCritic
     if (parameter_dictionary->HasKey("CostCritic")) {
         auto dict = parameter_dictionary->GetDictionary("CostCritic");
         if (dict) {
             auto* critic = options.mutable_cost_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("critical_cost")) {
                 critic->set_critical_cost(dict->GetDouble("critical_cost"));
             }
             if (dict->HasKey("consider_footprint")) {
                 critic->set_consider_footprint(
                     dict->GetBool("consider_footprint"));
             }
             if (dict->HasKey("collision_cost")) {
                 critic->set_collision_cost(dict->GetDouble("collision_cost"));
             }
             if (dict->HasKey("near_goal_distance")) {
                 critic->set_near_goal_distance(
                     dict->GetDouble("near_goal_distance"));
             }
             if (dict->HasKey("trajectory_point_step")) {
                 critic->set_trajectory_point_step(
                     dict->GetInt("trajectory_point_step"));
             }
         }
     }
 
     // PathAlignCritic
     if (parameter_dictionary->HasKey("PathAlignCritic")) {
         auto dict = parameter_dictionary->GetDictionary("PathAlignCritic");
         if (dict) {
             auto* critic = options.mutable_path_align_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("max_path_occupancy_ratio")) {
                 critic->set_max_path_occupancy_ratio(
                     dict->GetDouble("max_path_occupancy_ratio"));
             }
             if (dict->HasKey("trajectory_point_step")) {
                 critic->set_trajectory_point_step(
                     dict->GetInt("trajectory_point_step"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
             if (dict->HasKey("offset_from_furthest")) {
                 critic->set_offset_from_furthest(
                     dict->GetInt("offset_from_furthest"));
             }
             if (dict->HasKey("use_path_orientations")) {
                 critic->set_use_path_orientations(
                     dict->GetBool("use_path_orientations"));
             }
         }
     }
 
     // PathFollowCritic
     if (parameter_dictionary->HasKey("PathFollowCritic")) {
         auto dict = parameter_dictionary->GetDictionary("PathFollowCritic");
         if (dict) {
             auto* critic = options.mutable_path_follow_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("offset_from_furthest")) {
                 critic->set_offset_from_furthest(
                     dict->GetInt("offset_from_furthest"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
         }
     }
 
     // PathAngleCritic
     if (parameter_dictionary->HasKey("PathAngleCritic")) {
         auto dict = parameter_dictionary->GetDictionary("PathAngleCritic");
         if (dict) {
             auto* critic = options.mutable_path_angle_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("offset_from_furthest")) {
                 critic->set_offset_from_furthest(
                     dict->GetInt("offset_from_furthest"));
             }
             if (dict->HasKey("threshold_to_consider")) {
                 critic->set_threshold_to_consider(
                     dict->GetDouble("threshold_to_consider"));
             }
             if (dict->HasKey("max_angle_to_furthest")) {
                 critic->set_max_angle_to_furthest(
                     dict->GetDouble("max_angle_to_furthest"));
             }
             if (dict->HasKey("forward_preference")) {
                 critic->set_forward_preference(
                     dict->GetBool("forward_preference"));
             }
         }
     }
 
     // ObstaclesCritic
     if (parameter_dictionary->HasKey("ObstaclesCritic")) {
         auto dict = parameter_dictionary->GetDictionary("ObstaclesCritic");
         if (dict) {
             auto* critic = options.mutable_obstacles_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("repulsion_weight")) {
                 critic->set_repulsion_weight(
                     dict->GetDouble("repulsion_weight"));
             }
             if (dict->HasKey("critical_weight")) {
                 critic->set_critical_weight(dict->GetDouble("critical_weight"));
             }
             if (dict->HasKey("consider_footprint")) {
                 critic->set_consider_footprint(
                     dict->GetBool("consider_footprint"));
             }
             if (dict->HasKey("collision_cost")) {
                 critic->set_collision_cost(dict->GetDouble("collision_cost"));
             }
             if (dict->HasKey("collision_margin_distance")) {
                 critic->set_collision_margin_distance(
                     dict->GetDouble("collision_margin_distance"));
             }
             if (dict->HasKey("near_goal_distance")) {
                 critic->set_near_goal_distance(
                     dict->GetDouble("near_goal_distance"));
             }
         }
     }
 
     // VelocityDeadbandCritic
     if (parameter_dictionary->HasKey("VelocityDeadbandCritic")) {
         auto dict =
             parameter_dictionary->GetDictionary("VelocityDeadbandCritic");
         if (dict) {
             auto* critic = options.mutable_velocity_deadband_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("cost_power")) {
                 critic->set_cost_power(dict->GetInt("cost_power"));
             }
             if (dict->HasKey("cost_weight")) {
                 critic->set_cost_weight(dict->GetDouble("cost_weight"));
             }
             if (dict->HasKey("deadband_velocities")) {
                 auto velocities = dict->GetArrayValuesAsDoubles();
                 for (const auto& vel : velocities) {
                     critic->add_deadband_velocities(vel);
                 }
             }
         }
     }
 
     // TwirlingCritic
     if (parameter_dictionary->HasKey("TwirlingCritic")) {
         auto dict = parameter_dictionary->GetDictionary("TwirlingCritic");
         if (dict) {
             auto* critic = options.mutable_twirling_critic();
             if (dict->HasKey("enabled")) {
                 critic->set_enabled(dict->GetBool("enabled"));
             }
             if (dict->HasKey("twirling_cost_power")) {
                 critic->set_twirling_cost_power(
                     dict->GetInt("twirling_cost_power"));
             }
             if (dict->HasKey("twirling_cost_weight")) {
                 critic->set_twirling_cost_weight(
                     dict->GetDouble("twirling_cost_weight"));
             }
         }
     }
 
     return options;
 }
 
 }  // namespace tools
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy