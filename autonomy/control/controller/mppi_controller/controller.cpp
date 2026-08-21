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
 #include "autonomy/control/common/controller_exceptions.hpp"
 #include "autonomy/control/controller/mppi_controller/tools/mppi_options.hpp"
 #include "autonomy/control/proto/controller_options.pb.h"
 
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
     visualize_ = options_.visualize();
     publish_optimal_trajectory_ = false;
 
     // 在独立应用（如 examples/apps/mppi_controller_app.cpp）中，没有 ROS2
     // 节点， 这里使用空 Node 指针初始化 Optimizer 与 PathHandler，使其仍能依赖
     // proto 参数和 costmap 运行。
     auto parent_node = std::shared_ptr<autolink::Node>();  // nullptr
 
     optimizer_.initialize(parent_node, name_, costmap_wrapper_, &options_,
                           options.controller_frequency());
     path_handler_.initialize(parent_node, name_, costmap_wrapper_, tf_buffer_,
                              &options_);
 
     AINFO << "Configured MPPI Controller: " << name_
           << " (visualize=" << (visualize_ ? "true" : "false") << ")";
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
     const automsgs::msgs::geometry_msgs::PoseStamped& pose,
     const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
     automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
     common::GoalChecker* goal_checker, std::string& message) {
 #ifdef BENCHMARK_TESTING
     auto start = std::chrono::system_clock::now();
 #endif

  try {
    // Transform path first so closed-loop rolling goal uses the latest closest pose.
    automsgs::msgs::nav_msgs::Path transformed_plan =
        path_handler_.transformPath(pose);

    automsgs::msgs::geometry_msgs::Pose goal =
        path_handler_.getTransformedGoal(pose.header().stamp()).pose();

     if (!costmap_wrapper_ || !costmap_wrapper_->getCostmap()) {
         message = "MPPI: costmap unavailable";
         return proto::CONTROLLER_RESULT_MAP_ERROR;
     }

     map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
     std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock(
         *(costmap->getMutex()));

    automsgs::msgs::geometry_msgs::Twist robot_speed;
    *robot_speed.mutable_linear() = velocity.twist().linear();
    *robot_speed.mutable_angular() = velocity.twist().angular();

    last_robot_pose_ = pose;
    last_robot_velocity_ = robot_speed;
    last_goal_pose_ = goal;
    last_goal_checker_ = goal_checker;
    has_control_state_ = true;

    cmd_vel = optimizer_.evalControl(pose, robot_speed, transformed_plan, goal,
                                     goal_checker);

 #ifdef BENCHMARK_TESTING
     auto end = std::chrono::system_clock::now();
     auto duration =
         std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
             .count();
     AINFO << "Control loop execution time: " << duration << " [ms]";
 #endif

     if (visualize_ && trajectory_visualizer_.get()) {
         Eigen::ArrayXXf optimal_trajectory;
         Visualize(std::move(transformed_plan), cmd_vel.header().stamp(),
                   optimal_trajectory);
     }

     message.clear();
     return proto::CONTROLLER_RESULT_SUCCESS;
  } catch (const common::NoValidControl& ex) {
     message = ex.what();
     return proto::CONTROLLER_RESULT_NO_VALID_CMD;
  } catch (const common::InvalidPath& ex) {
     message = ex.what();
     return proto::CONTROLLER_RESULT_INVALID_PATH;
  } catch (const common::ControllerTFError& ex) {
     message = ex.what();
     return proto::CONTROLLER_RESULT_TF_ERROR;
  } catch (const common::ControllerException& ex) {
     message = ex.what();
     return proto::CONTROLLER_RESULT_FAILURE;
  }
 }
 
 void MPPIController::Visualize(
     automsgs::msgs::nav_msgs::Path transformed_plan,
     const automsgs::msgs::builtin_interfaces::Time& cmd_stamp,
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
 
void MPPIController::SetPlan(const automsgs::msgs::nav_msgs::Path& path) {
    path_handler_.setPath(path);
    has_control_state_ = false;
}
 
 void MPPIController::SetSpeedLimit(const double& speed_limit,
                                    const bool& percentage) {
     optimizer_.setSpeedLimit(speed_limit, percentage);
 }
 
bool MPPIController::IsGoalReached(double dist_tolerance,
                                   double angle_tolerance) {
    (void)dist_tolerance;
    (void)angle_tolerance;
    if (!has_control_state_ || last_goal_checker_ == nullptr) {
        return false;
    }
    return last_goal_checker_->IsGoalReached(last_robot_pose_.pose(),
                                             last_goal_pose_,
                                             last_robot_velocity_);
}
 
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy
 
 // Plugins
 CLASS_LOADER_REGISTER_CLASS(
     autonomy::control::controller::mppi_controller::MPPIController,
     autonomy::control::common::ControllerInterface)