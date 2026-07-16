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

 #pragma once

 #include <Eigen/Dense>
 #include <memory>
 #include <vector>
 
 #include "autonomy/common/macros.hpp"
 #include "autonomy/commsgs/geometry_msgs.hpp"
 #include "autonomy/control/common/goal_checker_interface.hpp"
 #include "autonomy/control/controller/mppi_controller/models/path.hpp"
 #include "autonomy/control/controller/mppi_controller/models/state.hpp"
 #include "autonomy/control/controller/mppi_controller/models/trajectories.hpp"
 #include "autonomy/control/controller/mppi_controller/motion_models.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 
 /**
  * @struct mppi::CriticData
  * @brief Data to pass to critics for scoring, including state, trajectories,
  * pruned path, global goal, costs, and important parameters to share
  */
 struct CriticData {
     const models::State& state;
     const models::Trajectories& trajectories;
     const models::Path& path;
     const commsgs::geometry_msgs::Pose& goal;
 
     Eigen::ArrayXf& costs;
     float& model_dt;
 
     bool fail_flag;
     common::GoalChecker* goal_checker;
     std::shared_ptr<mppi_controller::MotionModel> motion_model;
     std::optional<std::vector<bool>> path_pts_valid;
     std::optional<size_t> furthest_reached_path_point;
 };
 
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy