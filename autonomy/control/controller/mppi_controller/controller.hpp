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
 #include <string>
 
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
 #include "autonomy/control/common/controller_interface.hpp"
 #include "autonomy/control/controller/mppi_controller/models/constraints.hpp"
 #include "autonomy/control/controller/mppi_controller/optimizer.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/mppi_options.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/path_handler.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/trajectory_visualizer.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
 #include "autonomy/control/proto/mppi_controller.pb.h"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 
 /**
  * @class mppi::MPPIController
  * @brief Main plugin controller for MPPI Controller
  */
 class MPPIController : public common::ControllerInterface
 {
 public:
     /**
      * @brief Constructor for mppi::MPPIController
      */
     MPPIController() = default;
 
     /**
      * @brief Configure controller on bringup
      * @param parent WeakPtr to node
      * @param name Name of plugin
      * @param tf TF buffer to use
      * @param costmap_wrapper Costmap2DWrapper object of environment
      */
     void Configure(const proto::ControllerOptions& options, std::string name,
                    std::shared_ptr<autonomy::transform::Buffer> tf,
                    std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                        costmap_wrapper);
 
     /**
      * @brief Cleanup resources
      */
     void Cleanup();
 
     /**
      * @brief Activate controller
      */
     void Activate();
 
     /**
      * @brief Deactivate controller
      */
     void Deactivate();
 
     /**
      * @brief Reset the controller state between tasks
      */
     void Reset() override;
 
     /**
      * @brief Main method to compute velocities using the optimizer
      * @param robot_pose Robot pose
      * @param robot_speed Robot speed
      * @param goal_checker Pointer to the goal checker for awareness if
      * completed task
      */
     uint32 ComputeVelocityCommands(
         const automsgs::msgs::geometry_msgs::PoseStamped& pose,
         const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
         automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
         common::GoalChecker* goal_checker, std::string& message) override;
 
     /**
      * @brief Set new reference path to track
      * @param path Path to track
      */
     void SetPlan(const automsgs::msgs::planning_msgs::Path& path) override;
 
     /**
      * @brief Set new speed limit from callback
      * @param speed_limit Speed limit to use
      * @param percentage Bool if the speed limit is absolute or relative
      */
     void SetSpeedLimit(const double& speed_limit,
                        const bool& percentage) override;
 
     /**
      * @brief Check if goal is reached
      * @param dist_tolerance Distance tolerance
      * @param angle_tolerance Angle tolerance
      * @return True if goal is reached, false otherwise
      */
     bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;
 
     /**
      * @brief Candidate rollouts from the last ComputeVelocityCommands call.
      */
     const models::Trajectories& GetGeneratedTrajectories() {
         return optimizer_.getGeneratedTrajectories();
     }
 
     /**
      * @brief Optimized state trajectory (x, y, yaw) from the last control step.
      */
     Eigen::ArrayXXf GetOptimizedTrajectory() {
         return optimizer_.getOptimizedTrajectory();
     }
 
     const proto::MPPIControllerOptions& GetMppiOptions() const {
         return options_;
     }
 
 protected:
     /**
      * @brief Visualize trajectories
      * @param transformed_plan Transformed input plan
      * @param cmd_stamp Command stamp
      * @param optimal_trajectory Optimal trajectory, if already computed
      */
     void Visualize(automsgs::msgs::planning_msgs::Path transformed_plan,
                    const automsgs::msgs::builtin_interfaces::Time& cmd_stamp,
                    const Eigen::ArrayXXf& optimal_trajectory);
 
     std::string name_;
     std::shared_ptr<autolink::Node> parent_;
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
     std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
     std::shared_ptr<autolink::Writer<automsgs::msgs::planning_msgs::Path>>
         opt_traj_pub_;
 
     proto::MPPIControllerOptions options_;
     Optimizer optimizer_;
     tools::PathHandler path_handler_;
     std::unique_ptr<tools::TrajectoryVisualizer> trajectory_visualizer_;
 
    bool visualize_;
    bool publish_optimal_trajectory_;

    automsgs::msgs::geometry_msgs::PoseStamped last_robot_pose_;
    automsgs::msgs::geometry_msgs::Twist last_robot_velocity_;
    automsgs::msgs::geometry_msgs::Pose last_goal_pose_;
    common::GoalChecker* last_goal_checker_{nullptr};
    bool has_control_state_{false};
};
 
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy