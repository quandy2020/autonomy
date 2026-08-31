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
 * @file controller.hpp
 * @brief NMPC controller plugin (path tracking + collision-aware rollout check)
 */

#pragma once

#include <limits>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/controller/nmpc_controller/optimizer.hpp"
#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"
#include "autonomy/control/controller/nmpc_controller/path_transformer.hpp"
#include "autonomy/control/controller/nmpc_controller/trajectory_collision_checker.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::NMPCController
 * @brief Main plugin controller for NMPC differential-drive tracking
 *
 * Control loop:
 * 1. Transform/prune the global plan into the costmap frame.
 * 2. Project the robot onto the path and build horizon references.
 * 3. Solve the OCP (DDP / FMPC / C-GMRES) and validate the rollout on the costmap.
 * 4. Publish [v, omega] or zero velocity when the goal is reached.
 */
class NMPCController : public common::ControllerInterface {
 public:
    /**
     * @brief Constructor for nmpc_controller::NMPCController
     */
    NMPCController() = default;

    /**
     * @brief Configure controller on bringup
     * @param options Controller options from Lua/proto
     * @param name Name of plugin
     * @param tf_buffer TF buffer for path transformation
     * @param costmap_wrapper Costmap2DWrapper object of environment
     */
    void Configure(const proto::ControllerOptions& options, std::string name,
                   std::shared_ptr<transform::Buffer> tf_buffer,
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
     * @brief Main method to compute velocities using the NMPC optimizer
     * @param pose Robot pose
     * @param velocity Robot velocity
     * @param cmd_vel Output velocity command
     * @param goal_checker Pointer to the goal checker for task completion
     * @param message Human-readable status or error text
     */
    uint32 ComputeVelocityCommands(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
        automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    /**
     * @brief Set new reference path to track
     * @param plan Path to track
     */
    void SetPlan(const automsgs::msgs::nav_msgs::Path& plan) override;

    /**
     * @brief Set new speed limit from callback
     * @param speed_limit Speed limit to use
     * @param percentage Whether the speed limit is absolute or relative
     */
    void SetSpeedLimit(const double& speed_limit, const bool& percentage) override;

    /**
     * @brief Check if goal is reached
     * @param dist_tolerance Distance tolerance
     * @param angle_tolerance Angle tolerance
     * @return True if goal is reached, false otherwise
     */
    bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

 private:
    /**
     * @brief Wrap yaw to (-pi, pi]
     */
    static double NormalizeAngle(double yaw);

    /**
     * @brief Transform and load the local plan for the current robot pose
     * @param pose Current robot pose
     * @return True if a non-empty local plan is available
     */
    bool UpdateLocalPath(const automsgs::msgs::geometry_msgs::PoseStamped& pose);

    /**
     * @brief Update monotonic arc-length progress along the path
     * @param pose Current robot pose
     * @return Reference arc length used for horizon sampling
     */
    double UpdatePathProgress(const automsgs::msgs::geometry_msgs::Pose& pose);

    /**
     * @brief Cache pose, velocity, and goal data for IsGoalReached
     */
    void CacheControlState(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
        common::GoalChecker* goal_checker);

    /**
     * @brief Run NMPC and validate the predicted trajectory against the costmap
     * @param state Current state [x, y, yaw]
     * @param ref_s Reference arc length on the path
     * @param result Output command and rollout
     */
    bool SolveAndValidate(
        const DifferentialDriveProblem::StateVector& state, double ref_s,
        NmpcOptimizer::SolveResult* result);

    // Plugin instance name from controller server
    std::string plugin_name_;
    // NMPC options loaded at configure (before speed-limit overrides)
    proto::NMPCControllerOptions base_options_;
    // Active NMPC options (may reflect speed-limit scaling)
    proto::NMPCControllerOptions options_;
    // TF buffer used to transform the global plan into the costmap frame
    std::shared_ptr<transform::Buffer> tf_buffer_;
    // Local costmap wrapper for path cropping and collision checking
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    // Latest global plan received from SetPlan
    automsgs::msgs::nav_msgs::Path global_plan_;
    // Transforms and prunes global_plan_ into a local segment
    PathTransformer path_transformer_;
    // Arc-length path reference sampled for the NMPC horizon
    PathReference path_;
    // Validates predicted NMPC rollouts against the costmap
    TrajectoryCollisionChecker collision_checker_;
    // DDP / FMPC / C-GMRES optimizer facade
    std::unique_ptr<NmpcOptimizer> optimizer_;
    // max_linear_vel at configure time (baseline for percentage speed limits)
    double initial_max_linear_vel_ = 0.0;
    // Monotonic arc-length progress along the local path [m]
    double path_progress_s_ = 0.0;
    // Latest speed-limit value from SetSpeedLimit callback
    double speed_limit_ = 0.0;
    // True if speed_limit_ is a percentage of initial_max_linear_vel_
    bool speed_limit_percentage_ = false;
    // True after CacheControlState has run at least once this cycle
    bool has_control_state_ = false;
    // Last robot pose passed to ComputeVelocityCommands
    automsgs::msgs::geometry_msgs::PoseStamped last_robot_pose_;
    // Last robot velocity passed to ComputeVelocityCommands
    automsgs::msgs::geometry_msgs::Twist last_robot_velocity_;
    // Terminal pose of the current local path
    automsgs::msgs::geometry_msgs::Pose last_goal_pose_;
    // Goal checker used in the last control cycle (non-owning)
    common::GoalChecker* last_goal_checker_ = nullptr;
    // Planar distance to goal from the last cached robot pose [m]
    double last_dist_to_goal_ = std::numeric_limits<double>::infinity();
    // Heading error to goal from the last cached robot pose [rad]
    double last_angle_to_goal_ = std::numeric_limits<double>::infinity();
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
