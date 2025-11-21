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
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/class_loader/class_loader.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/common/progress_checker.hpp"
#include "autonomy/control/common/goal_checker.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/robot_utils.hpp"

namespace autonomy {
namespace control {

class ControllerServer 
{
public:

    using ControllerMap = std::unordered_map<std::string, common::ControllerInterface::SharedPtr>;
    using GoalCheckerMap = std::unordered_map<std::string, common::GoalChecker::SharedPtr>;
    using ProgressCheckerMap = std::unordered_map<std::string, common::ProgressChecker::SharedPtr>;

    /**
    * Define ControllerServer::SharedPtr type
    */
    AUTONOMY_SMART_PTR_DEFINITIONS(ControllerServer)

    /**
     * @brief A constructor for nautonomy::control::ControllerServer
     * @param options Additional options to control creation of the node.
     */
    ControllerServer(const proto::ControllerOptions& options);
    
    /**
     * @brief Destrructor for ControllerServer
     */
    ~ControllerServer();

    /**
     * @brief Starts planning tasks
     */
    void Start();

    /**
     * @brief Shutdown planning tasks
     */
    void WaitForShutdown();

protected:
    /**
     * @brief FollowPath action server callback. Handles action server updates and
     * spins server until goal is reached
     *
     * Provides global path to controller received from action client. Twist
     * velocities for the robot are calculated and published using controller at
     * the specified rate till the goal is reached.
     * @throw nav2_core::PlannerException
     */
    void ComputeControl();

    /**
     * @brief Find the valid controller ID name for the given request
     *
     * @param c_name The requested controller name
     * @param name Reference to the name to use for control if any valid available
     * @return bool Whether it found a valid controller to use
     */
    bool FindControllerId(const std::string& c_name, std::string& name);

    /**
     * @brief Find the valid goal checker ID name for the specified parameter
     *
     * @param c_name The goal checker name
     * @param name Reference to the name to use for goal checking if any valid available
     * @return bool Whether it found a valid goal checker to use
     */
    bool FindGoalCheckerId(const std::string& c_name, std::string& name);

    /**
     * @brief Find the valid progress checker ID name for the specified parameter
     *
     * @param c_name The progress checker name
     * @param name Reference to the name to use for progress checking if any valid available
     * @return bool Whether it found a valid progress checker to use
     */
    bool FindProgressCheckerId(const std::string& c_name, std::string& name);

    /**
     * @brief Assigns path to controller
     * @param path Path received from action server
     */
    void SetPlannerPath(const commsgs::planning_msgs::Path& path);
    
    /**
     * @brief Calculates velocity and publishes to "cmd_vel" topic
     */
    void ComputeAndPublishVelocity();
    
    /**
     * @brief Calls setPlannerPath method with an updated path received from
     * action server
     */
    void UpdateGlobalPath();

    /**
     * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
     * @param velocity Twist velocity to be published
     */
    void PublishVelocity(const commsgs::geometry_msgs::TwistStamped& velocity);

    /**
     * @brief Calls velocity publisher to publish zero velocity
     */
    void PublishZeroVelocity();

    /**
     * @brief Called on goal exit
     */
    void OnGoalExit();
    
    /**
     * @brief Checks if goal is reached
     * @return true or false
     */
    bool IsGoalReached();

    /**
     * @brief Obtain current pose of the robot
     * @param pose To store current pose of the robot
     * @return true if able to obtain current pose of the robot, else false
     */
    bool GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose);

    /**
     * @brief get the thresholded velocity
     * @param velocity The current velocity from odometry
     * @param threshold The minimum velocity to return non-zero
     * @return double velocity value
     */
    double GetThresholdedVelocity(double velocity, double threshold)
    {
        return (std::abs(velocity) > threshold) ? velocity : 0.0;
    }

    /**
     * @brief get the thresholded Twist
     * @param Twist The current Twist from odometry
     * @return Twist Twist after thresholds applied
     */
    commsgs::geometry_msgs::Twist2D GetThresholdedTwist(const commsgs::geometry_msgs::Twist2D & twist)
    {
        commsgs::geometry_msgs::Twist2D twist_thresh;
        twist_thresh.x = GetThresholdedVelocity(twist.x, min_x_velocity_threshold_);
        twist_thresh.y = GetThresholdedVelocity(twist.y, min_y_velocity_threshold_);
        twist_thresh.theta = GetThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
        return twist_thresh;
    }

    // The controller needs a costmap node
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    // std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

    // Publishers and subscribers
    // std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
    // std::unique_ptr<nav2_util::TwistPublisher> vel_publisher_;
    // rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

    // Progress Checker Plugin
    // pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
    ProgressCheckerMap progress_checkers_;
    std::vector<std::string> default_progress_checker_ids_;
    std::vector<std::string> default_progress_checker_types_;
    std::vector<std::string> progress_checker_ids_;
    std::vector<std::string> progress_checker_types_;
    std::string progress_checker_ids_concat_, current_progress_checker_;

    // Goal Checker Plugin
    // pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
    GoalCheckerMap goal_checkers_;
    std::vector<std::string> default_goal_checker_ids_;
    std::vector<std::string> default_goal_checker_types_;
    std::vector<std::string> goal_checker_ids_;
    std::vector<std::string> goal_checker_types_;
    std::string goal_checker_ids_concat_, current_goal_checker_;

    // Controller Plugins
    // pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
    ControllerMap controllers_;
    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> controller_ids_;
    std::vector<std::string> controller_types_;
    std::string controller_ids_concat_, current_controller_;

    double controller_frequency_;
    double min_x_velocity_threshold_;
    double min_y_velocity_threshold_;
    double min_theta_velocity_threshold_;

    double failure_tolerance_;
    bool use_realtime_priority_;
    bool publish_zero_velocity_;
    commsgs::builtin_interfaces::Duration costmap_update_timeout_;

    // Whether we've published the single controller warning yet
    commsgs::geometry_msgs::PoseStamped end_pose_;

    // Last time the controller generated a valid command
    commsgs::builtin_interfaces::Time last_valid_cmd_time_;

    // Current path container
    commsgs::planning_msgs::Path current_path_;

private:
    /**
     * @brief Callback for speed limiting messages
     * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
     */
    void SpeedLimitCallback(const commsgs::planning_msgs::SpeedLimit::SharedPtr msg);

    // controller options
    proto::ControllerOptions options_;
};

}  // namespace control
}  // namespace autonomy