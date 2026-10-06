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

/**
 * @file sando_controller.hpp
 * @brief Ground-robot SANDO plugin that turns a sampled polynomial into a body twist.
 *
 * The planner works in the world frame. This class reads the costmap, runs
 * SandoPlanner, and writes an automsgs TwistStamped in the body frame:
 * vx_b =  c yaw * vx_w + s yaw * vy_w, vy_b = -s yaw * vx_w + c yaw * vy_w.
 * Differential drive forces vy_b = 0 and may also zero vx_b when the lateral
 * component is large. Quadruped, humanoid, and wheeled-leg models keep both
 * linear channels.
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/sando_controller/sando_planner.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Which ground kinematics the twist writer assumes.
 *
 * The polynomial itself is the same for every model. Only the body-frame
 * projection of the next setpoint changes.
 */
enum class GroundMotionModel {
  kDiffDrive = 0,    ///< Nonholonomic. Body velocity_y is zero. A large lateral world velocity can also zero velocity_x.
  kQuadruped = 1,    ///< Holonomic plane. Body velocity_x and velocity_y are both commanded.
  kHumanoid = 2,     ///< Holonomic plane, same twist map as the quadruped.
  kWheeledLeg = 3,   ///< Holonomic plane, same twist map as the quadruped.
};

/**
 * @class SandoController
 * @brief ControllerInterface adapter for the ground SANDO local planner.
 *
 * Registration is the string alias "sando_controller" / "SandoController" in
 * ControllerServer. Options come from ControllerOptions.sando_controller_options.
 * The class does not subscribe to ROS topics: the plan is a nav_msgs Path,
 * the pose and velocity are PoseStamped and TwistStamped, and dynamic movers
 * are pushed in with AddDynamicObstacle.
 */
class SandoController : public common::ControllerInterface {
 public:
  SandoController() = default;
  ~SandoController() override = default;

  /**
   * @brief Load options, parse the motion model, and configure the planner.
   * @param options Parent controller options. The sando_controller_options field is copied.
   * @param name Plugin instance name, used only in the configure log line.
   * @param tf Transform buffer. Unused by the planar planner; kept for the interface.
   * @param costmap_wrapper Local costmap. Locked again on every velocity command.
   */
  void Configure(const proto::ControllerOptions& options, std::string name,
                 std::shared_ptr<transform::Buffer> tf,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

  /**
   * @brief Drop the stored path and reset the planner. The plugin becomes inactive.
   */
  void Cleanup();

  /**
   * @brief Mark the plugin active so ComputeVelocityCommands will plan.
   */
  void Activate();

  /**
   * @brief Mark the plugin inactive. Subsequent commands return not-initialized.
   */
  void Deactivate();

  /**
   * @brief Clear the committed plan, the yaw timer, and the speed scale.
   */
  void Reset() override;

  /**
   * @brief Ingest the costmap, advance one SANDO cycle, and write a body twist.
   *
   * Odometry twist is body-frame and is rotated into the world frame before
   * planning: vx_w = c yaw * vx_b - s yaw * vy_b. The command position is not
   * written; only the twist is. A short plan still tracks linear velocity and
   * only holds yaw.
   *
   * @param pose Current pose. Yaw is read with transform::tf2::getYaw.
   * @param velocity Current body twist.
   * @param cmd_vel Output body twist. Linear speed is scaled by SetSpeedLimit.
   * @param goal_checker Unused. Goal tests use the planner status plus the tolerances below.
   * @param message Failure text when the return code is not success.
   * @return SUCCESS, NO_VALID_CMD, BLOCKED_PATH, INVALID_PATH, or NOT_INITIALIZED.
   */
  uint32 ComputeVelocityCommands(
      const automsgs::msgs::geometry_msgs::PoseStamped& pose,
      const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
      automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
      common::GoalChecker* goal_checker, std::string& message) override;

  /**
   * @brief True only when the planner status is goal-reached and both tolerances hold.
   *
   * Hover avoidance keeps the status off goal-reached, so this stays false
   * while the robot is sliding away from a threat.
   *
   * @param dist_tolerance Acceptable planar distance to the terminal pose, meters.
   * @param angle_tolerance Acceptable absolute yaw error, radians.
   */
  bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

  /**
   * @brief Replace the global path and set the terminal goal to its last pose.
   * @param plan nav_msgs Path in the world frame. An empty path clears the goal.
   */
  void SetPlan(const automsgs::msgs::nav_msgs::Path& plan) override;

  /**
   * @brief Scale the outgoing linear speed.
   * @param speed_limit Absolute m/s when percentage is false, otherwise a fraction of max_linear_vel.
   * @param percentage True when speed_limit is a fraction in (0, 1].
   */
  void SetSpeedLimit(const double& speed_limit, const bool& percentage) override;

  /**
   * @brief Inject one mover that the costmap tracker must not overwrite.
   *
   * Stand-in for SANDO addTraj. Call it on the control thread before
   * ComputeVelocityCommands. Matching an existing obstacle within 0.4 m
   * refreshes that track instead of allocating a new id.
   *
   * @param x World x of the center, meters.
   * @param y World y of the center, meters.
   * @param velocity_x World x velocity, m/s.
   * @param velocity_y World y velocity, m/s.
   * @param radius Circumscribed radius, meters. The AABB half-extents are set to this radius.
   */
  void AddDynamicObstacle(double x, double y, double velocity_x, double velocity_y, double radius);

 private:
  proto::SandoControllerOptions controller_options_;  ///< Resolved options after ApplyDefaults.
  SandoPlanner planner_;                   ///< Planar SANDO loop.
  GroundMotionModel motion_model_{GroundMotionModel::kDiffDrive};  ///< Twist projection selected at configure time.
  automsgs::msgs::nav_msgs::Path plan_;    ///< Last path passed to SetPlan.
  mutable std::mutex plan_mutex_;          ///< Guards plan_, has_plan_, and the terminal goal.
  bool active_{false};                     ///< False until Activate, and after Deactivate or Cleanup.
  bool has_plan_{false};                   ///< True after a non-empty SetPlan.
  double speed_scale_{1.0};                ///< Multiplier applied to the commanded linear speed.
  double goal_distance_{1.0e9};                ///< Last planar distance from the robot to the terminal goal, meters.
  double goal_yaw_error_{0.0};               ///< Last wrapped yaw error, radians.
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
