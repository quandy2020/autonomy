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

 #include "autonomy/control/controller/mppi_controller/controller.hpp"

 #include "autonomy/common/configuration_file_resolver.hpp"
 #include "autonomy/common/lua_parameter_dictionary.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/mppi_options.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 
 void MPPIController::Configure(
     const proto::ControllerOptions& options, std::string name,
     std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
     name_ = name;
     tf_buffer_ = tf_buffer;
     costmap_wrapper_ = costmap_wrapper;
     options_ = options.mppi_controller_options();
 
     // 在独立应用（如 examples/apps/mppi_controller_app.cpp）中，没有 ROS2
     // 节点， 这里使用空 Node 指针初始化 Optimizer 与 PathHandler，使其仍能依赖
     // proto 参数和 costmap 运行。
     auto parent_node = std::shared_ptr<autolink::Node>();  // nullptr
 
     optimizer_.initialize(parent_node, name_, costmap_wrapper_, &options_);
     path_handler_.initialize(parent_node, name_, costmap_wrapper_, tf_buffer_,
                              &options_);
 
     AINFO << "Configured MPPI Controller: " << name_;
 }
 
 void MPPIController::Cleanup() {
     optimizer_.shutdown();
     opt_traj_pub_.reset();
     AINFO << "Cleaned up MPPI Controller: " << name_.c_str();
 }
 
 void MPPIController::Activate() {
     AINFO << "Activated MPPI Controller: ", name_.c_str();
 }
 
 void MPPIController::Deactivate() {
     AINFO << "Deactivated MPPI Controller: ", name_.c_str();
 }
 
 void MPPIController::Reset() {
     optimizer_.reset(
         false /*Don't reset zone-based speed limits between requests*/);
 }
 
 uint32 MPPIController::ComputeVelocityCommands(
     const commsgs::geometry_msgs::PoseStamped& pose,
     const commsgs::geometry_msgs::TwistStamped& velocity,
     commsgs::geometry_msgs::TwistStamped& cmd_vel,
     common::GoalChecker* goal_checker, std::string& message) {
 #ifdef BENCHMARK_TESTING
     auto start = std::chrono::system_clock::now();
 #endif
 
     // No mutex needed for proto options (read-only after Configure)
     commsgs::geometry_msgs::Pose goal =
         path_handler_.getTransformedGoal(pose.header.stamp).pose;
 
     commsgs::planning_msgs::Path transformed_plan =
         path_handler_.transformPath(pose);
 
     map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
     std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock(
         *(costmap->getMutex()));
 
     commsgs::geometry_msgs::Twist robot_speed;
     robot_speed.linear = velocity.twist.linear;
     robot_speed.angular = velocity.twist.angular;
     cmd_vel = optimizer_.evalControl(pose, robot_speed, transformed_plan, goal,
                                      goal_checker);
 
 #ifdef BENCHMARK_TESTING
     auto end = std::chrono::system_clock::now();
     auto duration =
         std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
             .count();
     AINFO << "Control loop execution time: " << duration << " [ms]";
 #endif
 
     // TODO: Trajectory type not available, need to define or use alternative
     // Eigen::ArrayXXf optimal_trajectory;
     // if (publish_optimal_trajectory_ &&
     //     opt_traj_pub_->get_subscription_count() > 0) {
     //     optimal_trajectory = optimizer_.getOptimizedTrajectory();
     //     auto trajectory_msg = utils::toTrajectoryMsg(
     //         optimal_trajectory, optimizer_.getOptimalControlSequence(),
     //         optimizer_.getSettings().model_dt, cmd.header);
     //     opt_traj_pub_->publish(std::move(trajectory_msg));
     // }
 
     if (visualize_ && trajectory_visualizer_.get()) {
         Eigen::ArrayXXf optimal_trajectory;
         Visualize(std::move(transformed_plan), cmd_vel.header.stamp,
                   optimal_trajectory);
     }
 
     // Return success code
     return 0;  // CONTROLLER_RESULT_SUCCESS
 }
 
 void MPPIController::Visualize(
     commsgs::planning_msgs::Path transformed_plan,
     const commsgs::builtin_interfaces::Time& cmd_stamp,
     const Eigen::ArrayXXf& optimal_trajectory) {
     if (trajectory_visualizer_) {
         trajectory_visualizer_->add(optimizer_.getGeneratedTrajectories(),
                                     "Candidate Trajectories");
         if (optimal_trajectory.size() > 0) {
             trajectory_visualizer_->add(optimal_trajectory,
                                         "Optimal Trajectory", cmd_stamp);
         } else {
             trajectory_visualizer_->add(optimizer_.getOptimizedTrajectory(),
                                         "Optimal Trajectory", cmd_stamp);
         }
         trajectory_visualizer_->visualize(std::move(transformed_plan));
     }
 }
 
 void MPPIController::SetPlan(const commsgs::planning_msgs::Path& path) {
     path_handler_.setPath(path);
 }
 
 void MPPIController::SetSpeedLimit(const double& speed_limit,
                                    const bool& percentage) {
     optimizer_.setSpeedLimit(speed_limit, percentage);
 }
 
 bool MPPIController::IsGoalReached(double dist_tolerance,
                                    double angle_tolerance) {
     // Use the goal checker if available, otherwise use default behavior
     // This is a placeholder implementation - should use goal_checker if
     // provided
     return false;  // Default: goal not reached
 }
 
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy
 
 // Plugins
 CLASS_LOADER_REGISTER_CLASS(
     autonomy::control::controller::mppi_controller::MPPIController,
     autonomy::control::common::ControllerInterface)