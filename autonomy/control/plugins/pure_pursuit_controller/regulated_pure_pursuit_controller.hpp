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

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/plugins/pure_pursuit_controller/path_handler.hpp"
#include "autonomy/control/plugins/pure_pursuit_controller/collision_checker.hpp"
#include "autonomy/control/plugins/pure_pursuit_controller/regulation_functions.hpp"

namespace autonomy {
namespace control {
namespace plugins {

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public common::ControllerInterface
{
public:

    using TfBuffer = autonomy::transform::Buffer;

    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    RegulatedPurePursuitController() = default;

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    ~RegulatedPurePursuitController() override = default;

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void Configure(
        std::string name, std::shared_ptr<TfBuffer> tf,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) override;

    /**
     * @brief Cleanup controller state machine
     */
    void Cleanup() override;

    /**
     * @brief Activate controller state machine
     */
    void Activate() override;

    /**
     * @brief Deactivate controller state machine
     */
    void Deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel, 
        common::GoalChecker* goal_checker,
        std::string& message) override;

    bool Cancel() override;

    /**
     * @brief nav2_core setPlan - Sets the global plan
     * @param path The global plan
     */
    void SetPlan(const commsgs::planning_msgs::Path& path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void SetSpeedLimit(const double& speed_limit, const bool& percentage) override;

    void Reset() override;

protected:
    /**
     * @brief Get lookahead distance
     * @param cmd the current speed to use to compute lookahead point
     * @return lookahead distance
     */
    double getLookAheadDistance(const commsgs::geometry_msgs::Twist &);

    /**
     * @brief Creates a PointStamped message for visualization
     * @param carrot_pose Input carrot point as a PoseStamped
     * @return CarrotMsg a carrot point marker, PointStamped
     */
    std::unique_ptr<commsgs::geometry_msgs::PointStamped> createCarrotMsg(
        const commsgs::geometry_msgs::PoseStamped& carrot_pose);

    /**
     * @brief Whether robot should rotate to rough path heading
     * @param carrot_pose current lookahead point
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @param x_vel_sign Velocoty sign (forward or backward)
     * @return Whether should rotate to path heading
     */
    bool shouldRotateToPath(
        const commsgs::geometry_msgs::PoseStamped& carrot_pose, double& angle_to_path,
        double& x_vel_sign);

    /**
     * @brief Whether robot should rotate to final goal orientation
     * @param carrot_pose current lookahead point
     * @return Whether should rotate to goal heading
     */
    bool shouldRotateToGoalHeading(const commsgs::geometry_msgs::PoseStamped & carrot_pose);

    /**
     * @brief Create a smooth and kinematically smoothed rotation command
     * @param linear_vel linear velocity
     * @param angular_vel angular velocity
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @param curr_speed the current robot speed
     */
    void rotateToHeading(
        double & linear_vel, double & angular_vel,
        const double & angle_to_path, const commsgs::geometry_msgs::Twist & curr_speed);

    /**
     * @brief apply regulation constraints to the system
     * @param linear_vel robot command linear velocity input
     * @param lookahead_dist optimal lookahead distance
     * @param curvature curvature of path
     * @param speed Speed of robot
     * @param pose_cost cost at this pose
     */
    void applyConstraints(
        const double & curvature, const commsgs::geometry_msgs::Twist & speed,
        const double & pose_cost, const commsgs::planning_msgs::Path & path,
        double & linear_vel, double & sign);

    /**
     * @brief Find the intersection a circle and a line segment.
     * This assumes the circle is centered at the origin.
     * If no intersection is found, a floating point error will occur.
     * @param p1 first endpoint of line segment
     * @param p2 second endpoint of line segment
     * @param r radius of circle
     * @return point of intersection
     */
    static commsgs::geometry_msgs::Point circleSegmentIntersection(
        const commsgs::geometry_msgs::Point & p1,
        const commsgs::geometry_msgs::Point & p2,
        double r);

    /**
     * @brief Get lookahead point
     * @param lookahead_dist Optimal lookahead distance
     * @param path Current global path
     * @param interpolate_after_goal If true, interpolate the lookahead point after the goal based
     * on the orientation given by the position of the last two pose of the path
     * @return Lookahead point
     */
    commsgs::geometry_msgs::PoseStamped getLookAheadPoint(
        const double&, const commsgs::planning_msgs::Path&,
        bool interpolate_after_goal = false);

    /**
     * @brief checks for the cusp position
     * @param pose Pose input to determine the cusp position
     * @return robot distance from the cusp
     */
    double findVelocitySignChange(const commsgs::planning_msgs::Path& transformed_plan);


    std::shared_ptr<TfBuffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    map::costmap_2d::Costmap2D* costmap_;

    Parameters * params_;
    double goal_dist_tol_;
    double control_duration_;
    bool cancelling_ = false;
    bool finished_cancelling_ = false;
    bool is_rotating_to_heading_ = false;

    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<commsgs::planning_msgs::Path>> global_path_pub_;
    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<commsgs::geometry_msgs::PointStamped>> carrot_pub_;
    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<commsgs::geometry_msgs::PointStamped>> curvature_carrot_pub_;
    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_rotating_to_heading_pub_;
    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<commsgs::planning_msgs::Path>> carrot_arc_pub_;
    std::unique_ptr<PathHandler> path_handler_;
    std::unique_ptr<ParameterHandler> param_handler_;
    std::unique_ptr<CollisionChecker> collision_checker_;
};

}  // namespace plugins
}  // namespace control
}  // namespace autonomy