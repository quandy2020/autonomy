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
#include <string>
 
#include "autolink/autolink.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autonomy/control/controller/mppi_controller/models/trajectories.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/transform/buffer.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace tools {
 
 /**
  * @class mppi::TrajectoryVisualizer
  * @brief Visualizes trajectories for debugging
  */
 class TrajectoryVisualizer
 {
 public:
     /**
      * @brief Constructor for mppi::TrajectoryVisualizer
      * @param parent Shared pointer to the node
      * @param name Name of the plugin
      * @param frame_id Frame to publish trajectories in
      * @param parameters_handler Parameter handler object
      */
     TrajectoryVisualizer(std::shared_ptr<autolink::Node> parent,
                          const std::string& name, const std::string& frame_id,
                          const proto::MPPIControllerOptions* options);
     /**
      * @brief Destructor for mppi::TrajectoryVisualizer
      */
     ~TrajectoryVisualizer();
 
     /**
      * @brief Add an optimal trajectory to visualize
      * @param trajectory Optimal trajectory
      */
     void add(const Eigen::ArrayXXf& trajectory,
              const std::string& marker_namespace,
              const automsgs::msgs::builtin_interfaces::Time& cmd_stamp);
 
     /**
      * @brief Add candidate trajectories to visualize
      * @param trajectories Candidate trajectories
      */
     void add(const models::Trajectories& trajectories,
              const std::string& marker_namespace);
 
     /**
      * @brief Visualize the plan
      * @param plan Plan to visualize
      */
     void visualize(const automsgs::msgs::nav_msgs::Path& plan);
 
     /**
      * @brief Reset object
      */
     void reset();
 
 protected:
     std::string frame_id_;
     std::shared_ptr<autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
         trajectories_publisher_;
     std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
         transformed_path_pub_;
     std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
         optimal_path_pub_;
 
     std::unique_ptr<automsgs::msgs::nav_msgs::Path> optimal_path_;
     std::unique_ptr<automsgs::msgs::visualization_msgs::MarkerArray> points_;
     int marker_id_ = 0;
 
     const proto::MPPIControllerOptions* options_;
 
     size_t trajectory_step_{0};
     size_t time_step_{0};
 };
 
 }  // namespace tools
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy