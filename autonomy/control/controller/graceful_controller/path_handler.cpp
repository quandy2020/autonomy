/*
 * Copyright 2025 The Openbot Authors
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

 #include "autonomy/control/controller/graceful_controller/path_handler.hpp"

 #include <algorithm>
 #include <limits>
 #include <memory>
 #include <string>
 #include <utility>
 #include <vector>
 
 #include "autonomy/control/common/controller_exceptions.hpp"
 #include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
 #include "autonomy/transform/tf2/convert.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 
 PathHandler::PathHandler(
     double transform_tolerance, std::shared_ptr<transform::Buffer> tf,
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper)
     : transform_tolerance_(transform_tolerance),
       tf_buffer_(tf),
       costmap_wrapper_(costmap_wrapper) {}
 
 commsgs::planning_msgs::Path PathHandler::TransformGlobalPlan(
     const commsgs::geometry_msgs::PoseStamped& pose,
     double max_robot_pose_search_dist) {
     // Check first if the plan is empty
     if (global_plan_.poses.empty()) {
         throw autonomy::control::common::InvalidPath(
             "Received plan with zero length");
     }
 
     // Let's get the pose of the robot in the frame of the plan
     commsgs::geometry_msgs::PoseStamped robot_pose;
     try {
         // Use Buffer's transform method directly
         robot_pose =
             tf_buffer_->transform(pose, global_plan_.header.frame_id,
                                   static_cast<float>(transform_tolerance_));
     } catch (const std::exception& e) {
         throw autonomy::control::common::ControllerTFError(
             "Unable to transform robot pose into global plan's frame: " +
             std::string(e.what()));
     }
 
     // Find the first pose in the global plan that's further than
     // max_robot_pose_search_dist from the robot using integrated distance Find
     // the first pose in the global plan that's further than
     // max_robot_pose_search_dist from the robot using integrated distance
     auto closest_pose_upper_bound = global_plan_.poses.end();
     double accumulated_dist = 0.0;
     for (auto it = global_plan_.poses.begin();
          it != global_plan_.poses.end() - 1; ++it) {
         accumulated_dist +=
             map::costmap_2d::utils::euclidean_distance(*it, *(it + 1));
         if (accumulated_dist > max_robot_pose_search_dist) {
             closest_pose_upper_bound = it + 1;
             break;
         }
     }
 
     // First find the closest pose on the path to the robot
     // bounded by when the path turns around (if it does) so we don't get a pose
     // from a later portion of the path
     auto transformation_begin = map::costmap_2d::utils::min_by(
         global_plan_.poses.begin(), closest_pose_upper_bound,
         [&robot_pose](const commsgs::geometry_msgs::PoseStamped& ps) {
             return map::costmap_2d::utils::euclidean_distance(robot_pose, ps);
         });
 
     // We'll discard points on the plan that are outside the local costmap
     auto* costmap = costmap_wrapper_->getCostmap();
     double dist_threshold =
         std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) /
         2.0;
     auto transformation_end = std::find_if(
         transformation_begin, global_plan_.poses.end(),
         [&](const auto& global_plan_pose) {
             return map::costmap_2d::utils::euclidean_distance(
                        global_plan_pose, robot_pose) > dist_threshold;
         });
 
     // Lambda to transform a PoseStamped from global frame to local
     auto TransformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
         commsgs::geometry_msgs::PoseStamped stamped_pose, transformed_pose;
         stamped_pose.header.frame_id = global_plan_.header.frame_id;
         stamped_pose.header.stamp = robot_pose.header.stamp;
         stamped_pose.pose = global_plan_pose.pose;
         try {
             // Use Buffer's transform method directly
             transformed_pose = tf_buffer_->transform(
                 stamped_pose, costmap_wrapper_->getGlobalFrameID(),
                 static_cast<float>(transform_tolerance_));
         } catch (const std::exception& e) {
             // Return empty pose to skip this pose (will be filtered out)
             transformed_pose.header.frame_id = "";  // Mark as invalid
             return transformed_pose;
         }
         transformed_pose.pose.position.z = 0.0;
         return transformed_pose;
     };
 
     // Transform the near part of the global plan into the robot's frame of
     // reference.
     commsgs::planning_msgs::Path transformed_plan;
     transformed_plan.header.frame_id =
         costmap_wrapper_->getGlobalFrameID();  // Use global frame as base frame
     transformed_plan.header.stamp = robot_pose.header.stamp;
     for (auto it = transformation_begin; it != transformation_end; ++it) {
         auto transformed = TransformGlobalPoseToLocal(*it);
         if (!transformed.header.frame_id.empty()) {
             transformed_plan.poses.push_back(transformed);
         }
         // Skip poses that couldn't be transformed (empty frame_id)
     }
 
     // Remove the portion of the global plan that we've already passed so we
     // don't process it on the next iteration (this is called path pruning)
     global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
 
     if (transformed_plan.poses.empty()) {
         throw autonomy::control::common::InvalidPath(
             "Resulting plan has 0 poses in it.");
     }
 
     return transformed_plan;
 }
 
 void PathHandler::SetPlan(const commsgs::planning_msgs::Path& path) {
     global_plan_ = path;
 }
 
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy