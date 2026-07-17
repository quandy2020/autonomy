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
 #include <memory>
 #include <mutex>
 #include <string>
 #include <vector>
 
 #include "autolink/autolink.hpp"
 #include "autonomy/commsgs/geometry_msgs.hpp"
 #include "autonomy/commsgs/planning_msgs.hpp"
 #include "autonomy/control/common/controller_exceptions.hpp"
 #include "autonomy/control/proto/pure_pursuit_controller.pb.h"
 #include "autonomy/control/utils/odometry_utils.hpp"
 #include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
 #include "autonomy/map/costmap_2d/filters/filter_values.hpp"
 #include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
 #include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace pure_pursuit_controller {
 
 /**
  * @class nav2_regulated_pure_pursuit_controller::CollisionChecker
  * @brief Checks for collision based on a RPP control command
  */
 class CollisionChecker
 {
 public:
     /**
      * @brief Constructor for
      * nav2_regulated_pure_pursuit_controller::CollisionChecker
      */
     CollisionChecker(
         std::shared_ptr<::autolink::Node> node,
         std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
         const proto::PurePursuitControllerOptions* options);
 
     /**
      * @brief Destrructor for
      * nav2_regulated_pure_pursuit_controller::CollisionChecker
      */
     ~CollisionChecker() = default;
 
     /**
      * @brief Whether collision is imminent
      * @param robot_pose Pose of robot
      * @param carrot_pose Pose of carrot
      * @param linear_vel linear velocity to forward project
      * @param angular_vel angular velocity to forward project
      * @param carrot_dist Distance to the carrot for PP
      * @return Whether collision is imminent
      */
     bool isCollisionImminent(const commsgs::geometry_msgs::PoseStamped&,
                              const double&, const double&, const double&);
 
     /**
      * @brief checks for collision at projected pose
      * @param x Pose of pose x
      * @param y Pose of pose y
      * @param theta orientation of Yaw
      * @return Whether in collision
      */
     bool inCollision(const double& x, const double& y, const double& theta);
 
     /**
      * @brief Cost at a point
      * @param x Pose of pose x
      * @param y Pose of pose y
      * @return Cost of pose in costmap
      */
     double costAtPose(const double& x, const double& y);
 
 protected:
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
     map::costmap_2d::Costmap2D* costmap_;
     std::unique_ptr<
         map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>>
         footprint_collision_checker_;
     const proto::PurePursuitControllerOptions* options_ = nullptr;
     std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>>
         carrot_arc_pub_;
 };
 
 }  // namespace pure_pursuit_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy