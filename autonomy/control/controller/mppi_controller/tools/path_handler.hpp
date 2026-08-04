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

#include <google/protobuf/repeated_ptr_field.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
 
#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace tools {
 
 using PathIterator = ::google::protobuf::RepeatedPtrField<
         automsgs::msgs::geometry_msgs::PoseStamped>::iterator;
 using PathRange = std::pair<PathIterator, PathIterator>;
 
 /**
  * @class mppi::PathHandler
  * @brief Manager of incoming reference paths for transformation and processing
  */
 
 class PathHandler
 {
 public:
     /**
      * @brief Constructor for mppi::PathHandler
      */
     PathHandler() = default;
 
     /**
      * @brief Destructor for mppi::PathHandler
      */
     ~PathHandler() = default;
 
     /**
      * @brief Initialize path handler on bringup
      * @param parent WeakPtr to node
      * @param name Name of plugin
      * @param costmap_ros Costmap2DROS object of environment
      * @param tf TF buffer for transformations
      * @param dynamic_parameter_handler Parameter handler object
      */
     void initialize(std::shared_ptr<autolink::Node> parent,
                     const std::string& name,
                     std::shared_ptr<map::costmap_2d::Costmap2DWrapper>,
                     std::shared_ptr<autonomy::transform::Buffer>,
                     const proto::MPPIControllerOptions*);
 
     /**
      * @brief Set new reference path
      * @param Plan Path to use
      */
     void setPath(const automsgs::msgs::nav_msgs::Path& plan);
 
     /**
      * @brief Get reference path
      * @return Path
      */
     automsgs::msgs::nav_msgs::Path& getPath();
 
     /**
      * @brief transform global plan to local applying constraints,
      * then prune global plan
      * @param robot_pose Pose of robot
      * @return global plan in local frame
      */
     automsgs::msgs::nav_msgs::Path transformPath(
         const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose);
 
     /**
      * @brief Get the global goal pose transformed to the local frame
      * @param stamp Time to get the goal pose at
      * @return Transformed goal pose
      */
     automsgs::msgs::geometry_msgs::PoseStamped getTransformedGoal(
         const automsgs::msgs::builtin_interfaces::Time& stamp);
 
 protected:
     /**
      * @brief Transform a pose to another frame
      * @param frame Frame to transform to
      * @param in_pose Input pose
      * @param out_pose Output pose
      * @return Bool if successful
      */
     bool transformPose(const std::string& frame,
                        const automsgs::msgs::geometry_msgs::PoseStamped& in_pose,
                        automsgs::msgs::geometry_msgs::PoseStamped& out_pose) const;
 
     /**
      * @brief Get largest dimension of costmap (radially)
      * @return Max distance from center of costmap to edge
      */
     double getMaxCostmapDist();
 
     /**
      * @brief Transform a pose to the global reference frame
      * @param pose Current pose
      * @return output poose in global reference frame
      */
     automsgs::msgs::geometry_msgs::PoseStamped transformToGlobalPlanFrame(
         const automsgs::msgs::geometry_msgs::PoseStamped& pose);
 
     /**
      * @brief Get global plan within window of the local costmap size
      * @param global_pose Robot pose
      * @return plan transformed in the costmap frame and iterator to the first
      * pose of the global plan (for pruning)
      */
     std::pair<automsgs::msgs::nav_msgs::Path, PathIterator>
     getGlobalPlanConsideringBoundsInCostmapFrame(
         const automsgs::msgs::geometry_msgs::PoseStamped& global_pose);
 
     /**
      * @brief Prune a path to only interesting portions
      * @param plan Plan to prune
      * @param end Final path iterator
      */
     void prunePlan(automsgs::msgs::nav_msgs::Path& plan, const PathIterator end);
 
     /**
      * @brief Check if the robot pose is within the set inversion tolerances
      * @param robot_pose Robot's current pose to check
      * @return bool If the robot pose is within the set inversion tolerances
      */
     bool isWithinInversionTolerances(
         const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose);
 
     std::string name_;
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
     std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
     const proto::MPPIControllerOptions* options_;
 
     automsgs::msgs::nav_msgs::Path global_plan_;
     automsgs::msgs::nav_msgs::Path global_plan_up_to_inversion_;
 
     double max_robot_pose_search_dist_{0};
     double prune_distance_{0};
     double transform_tolerance_{0};
     float inversion_xy_tolerance_{0.2};
     float inversion_yaw_tolerance{0.4};
     bool enforce_path_inversion_{false};
     unsigned int inversion_locale_{0u};

     /** Start/end poses are spatially close (e.g. closed rectangle/circle lap). */
     bool closed_loop_path_{false};
     /** Virtual goal distance ahead of closest pose on closed loops (m). */
     double closed_loop_goal_lookahead_{3.0};
     /** Closest path iterator from the last transformPath (valid for closed loops). */
     PathIterator last_closest_point_{};
 };
 
 }  // namespace tools
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy