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

#include "autonomy/control/controller/mppi_controller/tools/trajectory_visualizer.hpp"

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
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autonomy/transform/tf2/LinearMath/Quaternion.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace tools {
 
 TrajectoryVisualizer::TrajectoryVisualizer(
     std::shared_ptr<autolink::Node> parent, const std::string& name,
     const std::string& frame_id, const proto::MPPIControllerOptions* options) {
     frame_id_ = frame_id;
     trajectories_publisher_ =
         parent->CreateWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
             name + "/candidate_trajectories");
     transformed_path_pub_ = parent->CreateWriter<automsgs::msgs::nav_msgs::Path>(
         name + "/transformed_global_plan");
     optimal_path_pub_ = parent->CreateWriter<automsgs::msgs::nav_msgs::Path>(
         name + "/optimal_path");
     options_ = options;
 
     // Load TrajectoryVisualizer parameters from proto
     if (options && options->has_trajectory_visualizer()) {
         trajectory_step_ = options->trajectory_visualizer().trajectory_step();
         time_step_ = options->trajectory_visualizer().time_step();
     } else {
         trajectory_step_ = 5;  // Default
         time_step_ = 3;        // Default
     }
 
     reset();
 }
 
 void TrajectoryVisualizer::add(
     const Eigen::ArrayXXf& trajectory, const std::string& marker_namespace,
     const automsgs::msgs::builtin_interfaces::Time& cmd_stamp) {
     size_t size = trajectory.rows();
     if (!size) {
         return;
     }
 
     auto add_marker = [&](auto i) {
         float component = static_cast<float>(i) / static_cast<float>(size);
 
         auto pose = tools::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
         auto scale = i != size - 1 ? tools::createScale(0.03, 0.03, 0.07)
                                    : tools::createScale(0.07, 0.07, 0.09);
         auto color = tools::createColor(0, component, component, 1);
         auto marker = tools::createMarker(marker_id_++, pose, scale, color,
                                           frame_id_, marker_namespace);
         *points_->mutable_markers()->Add() = marker;
 
         // populate optimal path
         automsgs::msgs::geometry_msgs::PoseStamped pose_stamped;
         pose_stamped.mutable_header()->set_frame_id(frame_id_);
         *pose_stamped.mutable_pose() = pose;
 
         autonomy::transform::tf2::Quaternion quaternion_tf2;
         quaternion_tf2.setRPY(0., 0., trajectory(i, 2));
         pose_stamped.mutable_pose()->mutable_orientation()->set_x(quaternion_tf2.x());
         pose_stamped.mutable_pose()->mutable_orientation()->set_y(quaternion_tf2.y());
         pose_stamped.mutable_pose()->mutable_orientation()->set_z(quaternion_tf2.z());
         pose_stamped.mutable_pose()->mutable_orientation()->set_w(quaternion_tf2.w());
 
         *optimal_path_->mutable_poses()->Add() = pose_stamped;
     };
 
     *optimal_path_->mutable_header()->mutable_stamp() = cmd_stamp;
     optimal_path_->mutable_header()->set_frame_id(frame_id_);
     for (size_t i = 0; i < size; i++) {
         add_marker(i);
     }
 }
 
 void TrajectoryVisualizer::add(const models::Trajectories& trajectories,
                                const std::string& marker_namespace) {
     size_t n_rows = trajectories.x.rows();
     size_t n_cols = trajectories.x.cols();
     const float shape_1 = static_cast<float>(n_cols);
     points_->mutable_markers()->Reserve(floor(n_rows / trajectory_step_) *
                              floor(n_cols * time_step_));
 
     for (size_t i = 0; i < n_rows; i += trajectory_step_) {
         for (size_t j = 0; j < n_cols; j += time_step_) {
             const float j_flt = static_cast<float>(j);
             float blue_component = 1.0f - j_flt / shape_1;
             float green_component = j_flt / shape_1;
 
             auto pose = tools::createPose(trajectories.x(i, j),
                                           trajectories.y(i, j), 0.03);
             auto scale = tools::createScale(0.03, 0.03, 0.03);
             auto color =
                 tools::createColor(0, green_component, blue_component, 1);
             auto marker = tools::createMarker(marker_id_++, pose, scale, color,
                                               frame_id_, marker_namespace);
 
             *points_->mutable_markers()->Add() = marker;
         }
     }
 }
 
 void TrajectoryVisualizer::reset() {
     marker_id_ = 0;
     points_ = std::make_unique<automsgs::msgs::visualization_msgs::MarkerArray>();
     optimal_path_ = std::make_unique<automsgs::msgs::nav_msgs::Path>();
 }
 
 void TrajectoryVisualizer::visualize(const automsgs::msgs::nav_msgs::Path& plan) {
     if (trajectories_publisher_ && trajectories_publisher_->HasReader()) {
         auto msg = std::make_shared<automsgs::msgs::visualization_msgs::MarkerArray>(
             *points_);
         trajectories_publisher_->Write(msg);
     }
 
     if (optimal_path_pub_ && optimal_path_pub_->HasReader()) {
         auto msg =
             std::make_shared<automsgs::msgs::nav_msgs::Path>(*optimal_path_);
         optimal_path_pub_->Write(msg);
     }
 
     reset();
 
     if (transformed_path_pub_ && transformed_path_pub_->HasReader()) {
         auto plan_ptr = std::make_shared<automsgs::msgs::nav_msgs::Path>(plan);
         transformed_path_pub_->Write(plan_ptr);
     }
 }
 
 TrajectoryVisualizer::~TrajectoryVisualizer() = default;
 
 }  // namespace tools
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy