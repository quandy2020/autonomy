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

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/graceful_controller/path_handler.hpp"
#include "autonomy/control/controller/graceful_controller/smooth_control_law.hpp"
#include "autonomy/control/controller/graceful_controller/utils.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {

/**
 * @class nav2_graceful_controller::GracefulController
 * @brief Graceful controller plugin
 */
class GracefulController : public common::ControllerInterface
{
public:
    /**
     * @brief Constructor for nav2_graceful_controller::GracefulController
     */
    GracefulController() = default;

    /**
     * @brief Destructor for nav2_graceful_controller::GracefulController
     */
    ~GracefulController() override = default;

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void Configure(const proto::ControllerOptions& options, std::string name,
                   std::shared_ptr<transform::Buffer> tf,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;

    /**
     * @brief Cleanup controller state machine.
     */
    void Cleanup() override;

    /**
     * @brief Activate controller state machine.
     */
    void Activate() override;

    /**
     * @brief Deactivate controller state machine.
     */
    void Deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity.
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker Ptr to the goal checker for this task in case useful
     * in computing commands
     * @return          Best command
     */
    uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    /**
     * @brief Check if the goal pose has been achieved by the local planner
     * @param dist_tolerance The distance tolerance in which the current pose
     * will be partly accepted as reached goal
     * @param angle_tolerance The angle tolerance in which the current pose will
     * be partly accepted as reached goal
     * @return True if achieved, false otherwise
     */
    bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

    /**
     * @brief Set the global plan.
     * @param path The global plan
     */
    void SetPlan(const commsgs::planning_msgs::Path& path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed
     * @param percentage setting speed limit in percentage if true
     * or in absolute values in false case
     */
    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

protected:
    /**
     * @brief Validate a given target pose for calculating command velocity
     * @param target_pose Target pose to validate
     * @param dist_to_target Distance to target pose
     * @param dist_to_goal Distance to navigation goal
     * @param trajectory Trajectory to validate in simulation
     * @param costmap_transform Transform between global and local costmap
     * @param cmd_vel Initial command velocity to validate in simulation
     * @return true if target pose is valid, false otherwise
     */
    bool ValidateTargetPose(
        commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
        double dist_to_goal, commsgs::planning_msgs::Path& trajectory,
        commsgs::geometry_msgs::TransformStamped& costmap_transform,
        commsgs::geometry_msgs::TwistStamped& cmd_vel);

    /**
     * @brief Simulate trajectory calculating in every step the new velocity
     * command based on a new curvature value and checking for collisions.
     *
     * @param motion_target Motion target point (in costmap local frame?)
     * @param costmap_transform Transform between global and local costmap
     * @param trajectory Simulated trajectory
     * @param cmd_vel Initial command velocity during simulation
     * @param backward Flag to indicate if the robot is moving backward
     * @return true if the trajectory is collision free, false otherwise
     */
    bool SimulateTrajectory(
        const commsgs::geometry_msgs::PoseStamped& motion_target,
        const commsgs::geometry_msgs::TransformStamped& costmap_transform,
        commsgs::planning_msgs::Path& trajectory,
        commsgs::geometry_msgs::TwistStamped& cmd_vel, bool backward);

    /**
     * @brief Rotate the robot to face the motion target with maximum angular
     * velocity.
     *
     * @param angle_to_target Angle to the motion target
     * @return geometry_msgs::msg::Twist Velocity command
     */
    commsgs::geometry_msgs::Twist RotateToTarget(double angle_to_target);

    /**
     * @brief Checks if the robot is in collision
     * @param x The x coordinate of the robot in global frame
     * @param y The y coordinate of the robot in global frame
     * @param theta The orientation of the robot in global frame
     * @return Whether in collision
     */
    bool InCollision(const double& x, const double& y, const double& theta);

    /**
     * @brief Compute the distance to each pose in a path
     * @param poses Poses to compute distances with
     * @param distances Computed distances
     */
    void ComputeDistanceAlongPath(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
        std::vector<double>& distances);

    /**
     * @brief Control law requires proper orientations, not all planners provide
     * them
     * @param path Path to add orientations into, if required
     */
    void ValidateOrientations(
        std::vector<commsgs::geometry_msgs::PoseStamped>& path);

    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::string plugin_name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    std::unique_ptr<
        map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>>
        collision_checker_;

    double goal_dist_tolerance_;
    bool goal_reached_;

    // True from the time a new path arrives until we have completed an initial
    // rotation
    bool do_initial_rotation_;
    std::unique_ptr<PathHandler> path_handler_;
    std::unique_ptr<SmoothControlLaw> control_law_;
};

}  // namespace controller
}  // namespace control
}  // namespace autonomy