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

 #include <algorithm>
 #include <mutex>
 #include <string>
 
 #include "autonomy/control/proto/pure_pursuit_controller.pb.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace pure_pursuit_controller {
 
 /**
  * @class nav2_regulated_pure_pursuit_controller::ParameterHandler
  * @brief Handles parameters and dynamic parameters for RPP
  */
 class ParameterHandler
 {
 public:
     /**
      * @brief Constructor for
      * nav2_regulated_pure_pursuit_controller::ParameterHandler
      */
     explicit ParameterHandler(
         const proto::PurePursuitControllerOptions& options,
         const double costmap_size_x);
 
     /**
      * @brief Destrructor for
      * nav2_regulated_pure_pursuit_controller::ParameterHandler
      */
     ~ParameterHandler() = default;
 
     std::mutex& getMutex() {
         return mutex_;
     }
 
     // NOTE: Access to options is not internally synchronized. Caller should
     // hold getMutex().
     const proto::PurePursuitControllerOptions& GetOptions() const {
         return options_;
     }
 
     // Base value used to restore desired_linear_vel after a speed limit is
     // removed. NOTE: Caller should hold getMutex().
     double GetBaseDesiredLinearVel() const {
         return base_desired_linear_vel_;
     }
 
     // NOTE: Caller should hold getMutex().
     void RestoreBaseDesiredLinearVel() {
         options_.set_desired_linear_vel(base_desired_linear_vel_);
     }
 
     // NOTE: Caller should hold getMutex().
     void SetDesiredLinearVel(const double v) {
         options_.set_desired_linear_vel(v);
     }
 
     // Update parameters from proto options at runtime.
     // Returns true if applied successfully, false if rejected/adjusted due to
     // invalid inputs.
     bool Update(const proto::PurePursuitControllerOptions& options,
                 const double costmap_size_x, std::string* error);
 
 protected:
     // Dynamic parameters handler
     std::mutex mutex_;
     proto::PurePursuitControllerOptions options_;
     double base_desired_linear_vel_ = 0.0;
 };
 
 }  // namespace pure_pursuit_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy