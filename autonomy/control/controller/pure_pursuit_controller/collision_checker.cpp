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

 #include "autonomy/control/controller/pure_pursuit_controller/collision_checker.hpp"

 #include <cmath>
 #include <vector>
 
 #include "autolink/common/log.hpp"
 #include "autonomy/control/common/controller_exceptions.hpp"
 #include "autonomy/map/costmap_2d/cost_values.hpp"
 #include "autonomy/transform/tf2/utils.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace pure_pursuit_controller {
 
 CollisionChecker::CollisionChecker(
     std::shared_ptr<::autolink::Node> node,
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
     const proto::PurePursuitControllerOptions* options) {
     costmap_wrapper_ = costmap_wrapper;
     costmap_ = costmap_wrapper_->getCostmap();
     options_ = options;
 
     // initialize collision checker and set costmap
     footprint_collision_checker_ =
         std::make_unique<map::costmap_2d::FootprintCollisionChecker<
             map::costmap_2d::Costmap2D*>>(costmap_);
     footprint_collision_checker_->setCostmap(costmap_);
 
     // TODO: Create autolink writer for visualization
     // carrot_arc_pub_ =
     // node->CreateWriter<commsgs::planning_msgs::Path>("lookahead_collision_arc");
 }
 
 bool CollisionChecker::isCollisionImminent(
     const commsgs::geometry_msgs::PoseStamped& robot_pose,
     const double& linear_vel, const double& angular_vel,
     const double& carrot_dist) {
     // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
     // odom frame and the carrot_pose is in robot base frame. Just how the data
     // comes to us
 
     // check current point is OK
     if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y,
                     transform::tf2::getYaw(robot_pose.pose.orientation))) {
         return true;
     }
 
     // visualization messages
     commsgs::planning_msgs::Path arc_pts_msg;
     arc_pts_msg.header.frame_id = costmap_wrapper_->getGlobalFrameID();
     arc_pts_msg.header.stamp = robot_pose.header.stamp;
     commsgs::geometry_msgs::PoseStamped pose_msg;
     pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
     pose_msg.header.stamp = arc_pts_msg.header.stamp;
 
     double projection_time = 0.0;
     if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
         // rotating to heading at goal or toward path
         // Equation finds the angular distance required for the largest
         // part of the robot radius to move to another costmap cell:
         // theta_min = 2.0 * sin ((res/2) / r_max)
         // via isosceles triangle r_max-r_max-resolution,
         // dividing by angular_velocity gives us a timestep.
         double max_radius =
             costmap_wrapper_->getLayeredCostmap()->getCircumscribedRadius();
         projection_time = 2.0 *
                           sin((costmap_->getResolution() / 2) / max_radius) /
                           fabs(angular_vel);
     } else {
         // Normal path tracking
         if (fabs(linear_vel) < 1e-6) {
             // No motion, no imminent collision along an arc to check.
             if (carrot_arc_pub_) {
                 carrot_arc_pub_->Write(arc_pts_msg);
             }
             return false;
         }
         projection_time = costmap_->getResolution() / fabs(linear_vel);
     }
 
     const commsgs::geometry_msgs::Point& robot_xy = robot_pose.pose.position;
     commsgs::geometry_msgs::Pose2D curr_pose;
     curr_pose.x = robot_pose.pose.position.x;
     curr_pose.y = robot_pose.pose.position.y;
     curr_pose.theta = transform::tf2::getYaw(robot_pose.pose.orientation);
 
     // only forward simulate within time requested
     int i = 1;
     double max_allowed_time =
         1.0;  // Default, should come from
               // options_->max_allowed_time_to_collision_up_to_carrot
     if (options_) {
         max_allowed_time =
             options_->max_allowed_time_to_collision_up_to_carrot();
     }
     while (i * projection_time < max_allowed_time) {
         i++;
 
         // apply velocity at curr_pose over distance
         curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
         curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
         curr_pose.theta += projection_time * angular_vel;
 
         // check if past carrot pose, where no longer a thoughtfully valid
         // command
         if (std::hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) >
             carrot_dist) {
             break;
         }
 
         // store it for visualization
         pose_msg.pose.position.x = curr_pose.x;
         pose_msg.pose.position.y = curr_pose.y;
         pose_msg.pose.position.z = 0.01;
         arc_pts_msg.poses.push_back(pose_msg);
 
         // check for collision at the projected pose
         if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
             if (carrot_arc_pub_) {
                 carrot_arc_pub_->Write(arc_pts_msg);
             }
             return true;
         }
     }
 
     if (carrot_arc_pub_) {
         carrot_arc_pub_->Write(arc_pts_msg);
     }
 
     return false;
 }
 
 bool CollisionChecker::inCollision(const double& x, const double& y,
                                    const double& theta) {
     unsigned int mx, my;
 
     if (!costmap_->worldToMap(x, y, mx, my)) {
         AWARN << "The dimensions of the costmap is too small to successfully "
                  "check for "
               << "collisions as far ahead as requested. Proceed at your own "
                  "risk, slow the robot, or "
               << "increase your costmap size.";
         return false;
     }
 
     // Get robot footprint from costmap wrapper
     std::vector<commsgs::geometry_msgs::Point> footprint =
         costmap_wrapper_->getRobotFootprint();
     double footprint_cost = footprint_collision_checker_->footprintCostAtPose(
         x, y, theta, footprint);
     if (footprint_cost ==
             static_cast<double>(map::costmap_2d::NO_INFORMATION) &&
         costmap_wrapper_->getLayeredCostmap()->isTrackingUnknown()) {
         return false;
     }
 
     // if occupied or unknown and not to traverse unknown space
     return footprint_cost >=
            static_cast<double>(map::costmap_2d::LETHAL_OBSTACLE);
 }
 
 double CollisionChecker::costAtPose(const double& x, const double& y) {
     unsigned int mx, my;
 
     if (!costmap_->worldToMap(x, y, mx, my)) {
         AFATAL << "The dimensions of the costmap is too small to fully include "
                   "your robot's footprint, "
                << "thusly the robot cannot proceed further";
         throw autonomy::control::common::ControllerException(
             "RegulatedPurePursuitController: Dimensions of the costmap are too "
             "small "
             "to encapsulate the robot footprint at current speeds!");
     }
 
     unsigned char cost = costmap_->getCost(mx, my);
     return static_cast<double>(cost);
 }
 
 }  // namespace pure_pursuit_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy