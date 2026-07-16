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

 #include <cmath>
 
 #include "autonomy/common/macros.hpp"
 #include "autonomy/common/math/angle.hpp"
 #include "autonomy/commsgs/geometry_msgs.hpp"
 #include "autonomy/transform/tf2/utils.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 
 /**
  * @brief Egocentric polar coordinates defined as the difference between the
  * robot pose and the target pose relative to the robot position and
  * orientation.
  */
 struct EgocentricPolarCoordinates {
     float r;      // Radial distance between the robot pose and the target pose.
     float phi;    // Orientation of target with respect to the line of sight
                   // from the robot to the target.
     float delta;  // Steering angle of the robot with respect to the line of
                   // sight.
 
     EgocentricPolarCoordinates(const float& r_in = 0.0,
                                const float& phi_in = 0.0,
                                const float& delta_in = 0.0)
         : r(r_in), phi(phi_in), delta(delta_in) {}
 
     /**
      * @brief Construct a new egocentric polar coordinates as the difference
      * between the robot pose and the target pose relative to the robot position
      * and orientation, both referenced to the same frame.
      *
      * Thus, r, phi and delta are always at the origin of the frame.
      *
      * @param target Target pose.
      * @param current Current pose. Defaults to the origin.
      * @param backward If true, the robot is moving backwards. Defaults to
      * false.
      */
     explicit EgocentricPolarCoordinates(
         const commsgs::geometry_msgs::Pose& target,
         const commsgs::geometry_msgs::Pose& current =
             commsgs::geometry_msgs::Pose(),
         bool backward = false) {
         // Compute the difference between the target and the current pose
         float dX = target.position.x - current.position.x;
         float dY = target.position.y - current.position.y;
         // Compute the line of sight from the robot to the target
         // Flip it if the robot is moving backwards
         float line_of_sight =
             backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
         // Compute the ego polar coordinates
         r = sqrt(dX * dX + dY * dY);
         double target_yaw = transform::tf2::getYaw(target.orientation);
         double current_yaw = transform::tf2::getYaw(current.orientation);
         phi =
             common::math::NormalizeAngleDifference(target_yaw + line_of_sight);
         delta =
             common::math::NormalizeAngleDifference(current_yaw + line_of_sight);
     }
 };
 
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy