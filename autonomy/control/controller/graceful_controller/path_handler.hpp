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

 #include <memory>
 
 #include "autonomy/commsgs/geometry_msgs.hpp"
 #include "autonomy/commsgs/planning_msgs.hpp"
 #include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
 #include "autonomy/transform/buffer.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 
 /**
  * @class nav2_graceful_controller::PathHandler
  * @brief Handles input paths to transform them to local frames required
  */
 class PathHandler
 {
 public:
     /**
      * @brief Constructor for nav2_graceful_controller::PathHandler
      */
     PathHandler(
         double transform_tolerance, std::shared_ptr<transform::Buffer> tf,
         std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);
 
     /**
      * @brief Destructor for nav2_graceful_controller::PathHandler
      */
     ~PathHandler() = default;
 
     /**
      * @brief Transforms global plan into same frame as pose and clips poses
      * ineligible for motionTarget Points ineligible to be selected as a motion
      * target point if they are any of the following:
      * - Outside the local_costmap (collision avoidance cannot be assured)
      * @param pose pose to transform
      * @param max_robot_pose_search_dist Distance to search for matching nearest
      * path point
      * @return Path in new frame
      */
     commsgs::planning_msgs::Path TransformGlobalPlan(
         const commsgs::geometry_msgs::PoseStamped& pose,
         double max_robot_pose_search_dist);
 
     /**
      * @brief Sets the global plan
      *
      * @param path The global plan
      */
     void SetPlan(const commsgs::planning_msgs::Path& path);
 
     /**
      * @brief Gets the global plan
      *
      * @return The global plan
      */
     commsgs::planning_msgs::Path GetPlan() {
         return global_plan_;
     }
 
 protected:
     double transform_tolerance_{0};
     std::shared_ptr<transform::Buffer> tf_buffer_;
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
     commsgs::planning_msgs::Path global_plan_;
 };
 
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy