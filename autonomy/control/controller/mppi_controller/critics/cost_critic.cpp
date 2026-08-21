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

 #include "autonomy/control/controller/mppi_controller/critics/cost_critic.hpp"

 #include "autolink/common/log.hpp"
 #include "autonomy/control/common/controller_exceptions.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
 #include "autonomy/map/costmap_2d/cost_values.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace critics {
 
 void CostCritic::initialize() {
     if (!options_) {
         AWARN << "Options not set, using defaults";
         power_ = 1;
         weight_ = 3.81f / 254.0f;  // Normalized
         critical_cost_ = 300.0f;
         near_collision_cost_ = 253;
         collision_cost_ = 1000000.0f;
         near_goal_distance_ = 0.5f;
         inflation_layer_name_ = "";
         trajectory_point_step_ = 2;
         consider_footprint_ = false;
         return;
     }
 
     // Load from proto options
     if (options_->has_cost_critic()) {
         const auto& critic = options_->cost_critic();
         enabled_ = critic.enabled();
         power_ = critic.cost_power() > 0 ? critic.cost_power() : 1;
         const double raw_weight =
             critic.cost_weight() > 0.0 ? critic.cost_weight() : 3.81;
         weight_ = static_cast<float>(raw_weight) / 254.0f;  // Normalized
         critical_cost_ = critic.critical_cost() > 0.0
                              ? static_cast<float>(critic.critical_cost())
                              : 300.0f;
         collision_cost_ = critic.collision_cost() > 0.0
                               ? static_cast<float>(critic.collision_cost())
                               : 1000000.0f;
         near_goal_distance_ =
             critic.near_goal_distance() > 0.0
                 ? static_cast<float>(critic.near_goal_distance())
                 : 1.0f;
         trajectory_point_step_ = critic.trajectory_point_step() > 0
                                      ? critic.trajectory_point_step()
                                      : 2;
         consider_footprint_ = critic.consider_footprint();
         near_collision_cost_ = 253;
     } else {
         enabled_ = true;
         power_ = 1;
         weight_ = 3.81f / 254.0f;  // Normalized
         critical_cost_ = 300.0f;
         near_collision_cost_ = 253;
         collision_cost_ = 1000000.0f;
         near_goal_distance_ = 0.5f;
         trajectory_point_step_ = 2;
         consider_footprint_ = true;  // Default
     }
 
     costmap_ = costmap_ros_->getCostmap();
     collision_checker_.setCostmap(costmap_);
     possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
 
     if (possible_collision_cost_ < 1.0f) {
         AERROR << "Inflation layer either not found or inflation is not set "
                << "sufficiently for optimized non-circular collision checking "
                << "capabilities. It is HIGHLY recommended to set the inflation "
                << "radius to be at MINIMUM half of the robot's largest "
                << "cross-section.";
     }
 
     if (costmap_ros_->getUseRadius() == consider_footprint_) {
         AWARN << "Inconsistent configuration in collision checking. Please "
               << "verify the robot's shape settings in both the costmap and "
               << "the cost critic.";
         if (costmap_ros_->getUseRadius()) {
             throw common::ControllerException(
                 "Considering footprint in collision checking but no robot "
                 "footprint provided in the costmap.");
         }
     }
 
     if (near_collision_cost_ > map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
         AWARN << "Near collision cost is set higher than "
               << "INSCRIBED_INFLATED_OBSTACLE";
     }
 
     AINFO << "CostCritic instantiated with " << power_ << " power and "
           << critical_cost_ << " / " << weight_ << " weights. "
           << "Critic will collision check based on "
           << (consider_footprint_ ? "footprint" : "circular") << " cost.";
 }
 
 float CostCritic::findCircumscribedCost(
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
     double result = -1.0;
     const double circum_radius =
         costmap->getLayeredCostmap()->getCircumscribedRadius();
     if (static_cast<float>(circum_radius) == circumscribed_radius_) {
         // early return if footprint size is unchanged
         return circumscribed_cost_;
     }
 
     // check if the costmap has an inflation layer
     const auto inflation_layer =
         map::costmap_2d::InflationLayer::getInflationLayer(
             costmap, inflation_layer_name_);
     if (inflation_layer != nullptr) {
         const double resolution = costmap->getCostmap()->getResolution();
         double inflation_radius = inflation_layer->getInflationRadius();
         if (inflation_radius < circum_radius) {
             AERROR << "The inflation radius (" << inflation_radius
                    << ") is smaller than the circumscribed radius ("
                    << circum_radius << "). If this is an SE2-collision "
                    << "checking plugin, it cannot use costmap potential field "
                    << "to speed up collision checking by only checking the "
                    << "full footprint when robot is within possibly-inscribed "
                    << "radius of an obstacle. This may significantly slow "
                    << "down planning times!";
             result = 0.0;
             return result;
         }
         result = inflation_layer->computeCost(circum_radius / resolution);
     } else {
         AWARN << "No inflation layer found in costmap configuration. "
               << "If this is an SE2-collision checking plugin, it cannot use "
               << "costmap potential field to speed up collision checking by "
               << "only checking the full footprint when robot is within "
               << "possibly-inscribed radius of an obstacle. This may "
               << "significantly slow down planning times and not avoid "
               << "anything but absolute collisions!";
     }
 
     circumscribed_radius_ = static_cast<float>(circum_radius);
     circumscribed_cost_ = static_cast<float>(result);
 
     return circumscribed_cost_;
 }
 
 void CostCritic::score(CriticData& data) {
     if (!enabled_) {
         return;
     }
 
     // Rolling / rebuilt costmaps may replace the underlying Costmap2D*; always
     // refresh before scoring so collision checks see the latest obstacles.
     costmap_ = costmap_ros_->getCostmap();
     collision_checker_.setCostmap(costmap_);
 
     automsgs::msgs::geometry_msgs::Pose goal =
         tools::getCriticGoal(data, enforce_path_inversion_);
 
     // Setup cost information for various parts of the critic
     // Local rolling costmaps are mostly NO_INFORMATION until lasers clear
     // free space. Treating unknown as lethal makes every sample collide and
     // aborts FollowPath ("Optimizer fail to compute path").
     is_tracking_unknown_ = true;
     auto* costmap = collision_checker_.getCostmap();
     origin_x_ = static_cast<float>(costmap->getOriginX());
     origin_y_ = static_cast<float>(costmap->getOriginY());
     resolution_ = static_cast<float>(costmap->getResolution());
     size_x_ = costmap->getSizeInCellsX();
     size_y_ = costmap->getSizeInCellsY();
 
     if (consider_footprint_) {
         // footprint may have changed since initialization if user has dynamic
         // footprints
         possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
     }
 
     // If near the goal, don't apply the preferential term since the goal is
     // near obstacles
     bool near_goal = false;
     if (tools::withinPositionGoalTolerance(near_goal_distance_,
                                            data.state.pose.pose(), goal)) {
         near_goal = true;
     }
 
     Eigen::ArrayXf repulsive_cost(data.costs.rows());
     repulsive_cost.setZero();
     bool all_trajectories_collide = true;
 
     int strided_traj_cols =
         floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
     int strided_traj_rows = data.trajectories.x.rows();
     int outer_stride = strided_traj_rows * trajectory_point_step_;
 
     const auto traj_x =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
     const auto traj_y =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
     const auto traj_yaw =
         Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
             data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
             Eigen::Stride<-1, -1>(outer_stride, 1));
 
     for (int i = 0; i < strided_traj_rows; ++i) {
         bool trajectory_collide = false;
         float pose_cost = 0.0f;
         float& traj_cost = repulsive_cost(i);
 
         for (int j = 0; j < strided_traj_cols; j++) {
             float Tx = traj_x(i, j);
             float Ty = traj_y(i, j);
             unsigned int x_i = 0u, y_i = 0u;
 
             // The getCost doesn't use orientation
             // The footprintCostAtPose will always return "INSCRIBED" if
             // footprint is over it So the center point has more information
             // than the footprint
             if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
                // Outside the local window: soft penalty, not a hard collision.
                traj_cost += critical_cost_;
                continue;
            } else {
                pose_cost =
                    static_cast<float>(costmap->getCost(getIndex(x_i, y_i)));
                if (pose_cost < 1.0f) {
                    continue;  // In free space
                }
            }
 
             if (inCollision(pose_cost, Tx, Ty, traj_yaw(i, j))) {
                 traj_cost = collision_cost_;
                 trajectory_collide = true;
                 break;
             }
 
             // Let near-collision trajectory points be punished severely
             // Note that we collision check based on the footprint actual,
             // but score based on the center-point cost regardless
             if (pose_cost >= static_cast<float>(near_collision_cost_)) {
                 traj_cost += critical_cost_;
             } else if (!near_goal) {  // Generally prefer trajectories further
                                       // from obstacles
                 traj_cost += pose_cost;
             }
         }
 
         all_trajectories_collide &= trajectory_collide;
     }
 
     if (power_ > 1u) {
         data.costs +=
             (repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols)))
                 .pow(power_);
     } else {
         data.costs +=
             repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
     }
 
     data.fail_flag = all_trajectories_collide;
 }
 
 }  // namespace critics
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy
