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

 #include "autonomy/control/controller/mppi_controller/critics/path_align_critic.hpp"

 #include "autolink/common/log.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace critics {
 
 void PathAlignCritic::initialize() {
     if (!options_) {
         AWARN << "Options not set, using defaults";
         power_ = 1;
         weight_ = 14.0f;
         max_path_occupancy_ratio_ = 0.05f;
         offset_from_furthest_ = 20;
         trajectory_point_step_ = 4;
         threshold_to_consider_ = 0.5f;
         use_path_orientations_ = false;
         return;
     }
 
     // Load from proto options
     if (options_->has_path_align_critic()) {
         const auto& critic = options_->path_align_critic();
         enabled_ = critic.enabled();
         power_ = critic.cost_power();
         weight_ = static_cast<float>(critic.cost_weight());
         max_path_occupancy_ratio_ =
             static_cast<float>(critic.max_path_occupancy_ratio());
         offset_from_furthest_ =
             static_cast<size_t>(critic.offset_from_furthest());
         trajectory_point_step_ = critic.trajectory_point_step();
         threshold_to_consider_ =
             static_cast<float>(critic.threshold_to_consider());
         use_path_orientations_ = critic.use_path_orientations();
     } else {
         enabled_ = true;
         power_ = 1;
         weight_ = 14.0f;                    // Default
         max_path_occupancy_ratio_ = 0.05f;  // Default
         offset_from_furthest_ = 20;         // Default
         trajectory_point_step_ = 4;         // Default
         threshold_to_consider_ = 0.5f;      // Default
         use_path_orientations_ = false;     // Default
     }
 
     AINFO << "PathAlignCritic instantiated with " << power_ << " power and "
           << weight_ << " weight";
 }
 
 void PathAlignCritic::score(CriticData& data) {
     if (!enabled_) {
         return;
     }
 
     commsgs::geometry_msgs::Pose goal =
         tools::getCriticGoal(data, enforce_path_inversion_);
 
     // Don't apply close to goal, let the goal critics take over
     if (tools::withinPositionGoalTolerance(threshold_to_consider_,
                                            data.state.pose.pose, goal)) {
         return;
     }
 
     // Don't apply when first getting bearing w.r.t. the path
     tools::setPathFurthestPointIfNotSet(data);
     // Up to furthest only, closest path point is always 0 from path handler
     const size_t path_segments_count = *data.furthest_reached_path_point;
     float path_segments_flt = static_cast<float>(path_segments_count);
     if (path_segments_count < offset_from_furthest_) {
         return;
     }
 
     // Don't apply when dynamic obstacles are blocking significant proportions
     // of the local path
     tools::setPathCostsIfNotSet(data, costmap_ros_);
     std::vector<bool>& path_pts_valid = *data.path_pts_valid;
     float invalid_ctr = 0.0f;
     for (size_t i = 0; i < path_segments_count; i++) {
         if (!path_pts_valid[i]) {
             invalid_ctr += 1.0f;
         }
         if (invalid_ctr / path_segments_flt > max_path_occupancy_ratio_ &&
             invalid_ctr > 2.0f) {
             return;
         }
     }
 
     const size_t batch_size = data.trajectories.x.rows();
     Eigen::ArrayXf cost(data.costs.rows());
     cost.setZero();
 
     // Find integrated distance in the path
     std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
     std::vector<tools::Pose2D> path(path_segments_count);
     float dx = 0.0f, dy = 0.0f;
     for (unsigned int i = 1; i != path_segments_count; i++) {
         auto& pose = path[i - 1];
         pose.x = data.path.x(i - 1);
         pose.y = data.path.y(i - 1);
         pose.theta = data.path.yaws(i - 1);
 
         dx = data.path.x(i) - pose.x;
         dy = data.path.y(i) - pose.y;
         path_integrated_distances[i] =
             path_integrated_distances[i - 1] + sqrtf(dx * dx + dy * dy);
     }
 
     // Finish populating the path vector
     auto& final_pose = path[path_segments_count - 1];
     final_pose.x = data.path.x(path_segments_count - 1);
     final_pose.y = data.path.y(path_segments_count - 1);
     final_pose.theta = data.path.yaws(path_segments_count - 1);
 
     float summed_path_dist = 0.0f, dyaw = 0.0f;
     unsigned int num_samples = 0u;
     unsigned int path_pt = 0u;
     float traj_integrated_distance = 0.0f;
 
     int strided_traj_rows = data.trajectories.x.rows();
     int strided_traj_cols =
         floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
     int outer_stride = strided_traj_rows * trajectory_point_step_;
     // Get strided trajectory information
     const auto T_x =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
     const auto T_y =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
     const auto T_yaw =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
     const auto traj_sampled_size = T_x.cols();
 
     for (size_t t = 0; t < batch_size; ++t) {
         summed_path_dist = 0.0f;
         num_samples = 0u;
         traj_integrated_distance = 0.0f;
         path_pt = 0u;
         float Tx_m1 = T_x(t, 0);
         float Ty_m1 = T_y(t, 0);
         for (int p = 1; p < traj_sampled_size; p++) {
             const float Tx = T_x(t, p);
             const float Ty = T_y(t, p);
             dx = Tx - Tx_m1;
             dy = Ty - Ty_m1;
             Tx_m1 = Tx;
             Ty_m1 = Ty;
             traj_integrated_distance += sqrtf(dx * dx + dy * dy);
             path_pt = tools::findClosestPathPt(
                 path_integrated_distances, traj_integrated_distance, path_pt);
 
             // The nearest path point to align to needs to be not in collision,
             // else let the obstacle critic take over in this region due to
             // dynamic obstacles
             if (path_pts_valid[path_pt]) {
                 const auto& pose = path[path_pt];
                 dx = pose.x - Tx;
                 dy = pose.y - Ty;
                 num_samples++;
                 if (use_path_orientations_) {
                     dyaw = tools::shortest_angular_distance(pose.theta,
                                                             T_yaw(t, p));
                     summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
                 } else {
                     summed_path_dist += sqrtf(dx * dx + dy * dy);
                 }
             }
         }
         if (num_samples > 0u) {
             cost(t) = summed_path_dist / static_cast<float>(num_samples);
         } else {
             cost(t) = 0.0f;
         }
     }
 
     if (power_ > 1u) {
         data.costs += (cost * weight_).pow(power_).eval();
     } else {
         data.costs += (cost * weight_).eval();
     }
 }
 
 }  // namespace critics
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy
