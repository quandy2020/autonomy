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
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace models {
 
 /**
  * @class mppi::models::Trajectories
  * @brief Candidate Trajectories
  */
 struct Trajectories {
     Eigen::ArrayXXf x;
     Eigen::ArrayXXf y;
     Eigen::ArrayXXf yaws;
 
     /**
      * @brief Reset state data
      */
     void reset(unsigned int batch_size, unsigned int time_steps) {
         x.setZero(batch_size, time_steps);
         y.setZero(batch_size, time_steps);
         yaws.setZero(batch_size, time_steps);
     }
 };
 
 }  // namespace models
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy