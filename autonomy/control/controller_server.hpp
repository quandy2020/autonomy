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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/common/progress_checker_interface.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/robot_utils.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {

class ControllerServer
{
public:
    using ControllerMap =
        std::unordered_map<std::string, common::ControllerInterface::SharedPtr>;
    using GoalCheckerMap =
        std::unordered_map<std::string, common::GoalChecker::SharedPtr>;
    using ProgressCheckerMap =
        std::unordered_map<std::string, common::ProgressChecker::SharedPtr>;

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
    void Shutdown();

    enum class FollowPathTickResult {
        Running,
        Succeeded,
        Failed,
        Cancelled,
    };

    map::costmap_2d::Costmap2DWrapper::SharedPtr GetCostmapWrapper() const {
        return costmap_wrapper_;
    }

    /** TF + frames for in-process goal checks (set by TaskScheduler). */
    void SetNavigationContext(std::shared_ptr<transform::Buffer> tf_buffer,
                              const std::string& global_frame,
                              const std::string& robot_base_frame);

    /** Use planner global costmap when local costmap is disabled (single-process). */
    void SetSharedCostmap(
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    void SetOdomSmoother(std::shared_ptr<utils::OdomSmoother> odom_smoother);

    /**
     * @brief Begin following a path in-process (one control step per TickFollowPath).
     */
    bool BeginFollowPath(const commsgs::planning_msgs::Path& path,
                         const std::string& controller_id,
                         const std::string& goal_checker_id,
                         const std::string& progress_checker_id);

    FollowPathTickResult TickFollowPath(std::function<bool()> cancel_checker);

    void EndFollowPath();

    enum class RecoveryMotionType { Spin, BackUp, DriveOnHeading };

    enum class RecoveryTickResult { Running, Succeeded, Failed, Cancelled };

    struct RecoveryMotionCommand {
        RecoveryMotionType type{RecoveryMotionType::Spin};
        double distance{0.0};
        double speed{0.0};
        double time_allowance_sec{10.0};
        bool disable_collision_checks{false};
    };

    /** Begin an open-loop recovery motion (ends active FollowPath). */
    bool BeginRecoveryMotion(const RecoveryMotionCommand& cmd);

    RecoveryTickResult TickRecoveryMotion(std::function<bool()> cancel_checker);

    void EndRecoveryMotion();

    /** Last velocity command (in-process demo / mock integrator). */
    commsgs::geometry_msgs::TwistStamped GetLastCmdVel() const;

    /**
     * @brief Apply a dynamic speed limit to all loaded controller plugins.
     * @param msg speed_limit in m/s or percent per msg->percentage.
     */
    void ApplySpeedLimit(const commsgs::planning_msgs::SpeedLimit& msg);

protected:
    /**
     * @brief FollowPath action server callback. Handles action server updates
     * and spins server until goal is reached
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
     * @param name Reference to the name to use for control if any valid
     * available
     * @return bool Whether it found a valid controller to use
     */
    bool FindControllerId(const std::string& c_name, std::string& name);

    /**
     * @brief Find the valid goal checker ID name for the specified parameter
     *
     * @param c_name The goal checker name
     * @param name Reference to the name to use for goal checking if any valid
     * available
     * @return bool Whether it found a valid goal checker to use
     */
    bool FindGoalCheckerId(const std::string& c_name, std::string& name);

    /**
     * @brief Find the valid progress checker ID name for the specified
     * parameter
     *
     * @param c_name The progress checker name
     * @param name Reference to the name to use for progress checking if any
     * valid available
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
     * @brief Calls velocity publisher to publish the velocity on "cmd_vel"
     * topic
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

    void LoadPlugins();
    void ActivateControllers();
    void DeactivateControllers();

    common::ControllerInterface* GetController(const std::string& id);
    common::GoalChecker* GetGoalChecker(const std::string& id);
    common::ProgressChecker* GetProgressChecker(const std::string& id);

    /**
     * @brief get the thresholded velocity
     * @param velocity The current velocity from odometry
     * @param threshold The minimum velocity to return non-zero
     * @return double velocity value
     */
    double GetThresholdedVelocity(double velocity, double threshold) {
        return (std::abs(velocity) > threshold) ? velocity : 0.0;
    }

    /**
     * @brief get the thresholded Twist
     * @param Twist The current Twist from odometry
     * @return Twist Twist after thresholds applied
     */
    commsgs::geometry_msgs::Twist2D GetThresholdedTwist(
        const commsgs::geometry_msgs::Twist2D& twist) {
        commsgs::geometry_msgs::Twist2D twist_thresh;
        twist_thresh.x =
            GetThresholdedVelocity(twist.x, min_x_velocity_threshold_);
        twist_thresh.y =
            GetThresholdedVelocity(twist.y, min_y_velocity_threshold_);
        twist_thresh.theta =
            GetThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
        return twist_thresh;
    }

    // The controller needs a costmap node
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::string global_frame_{"map"};
    std::string robot_base_frame_{"base_link"};
    double goal_reached_tolerance_{0.25};
    std::unique_ptr<std::thread> costmap_thread_{nullptr};

    std::shared_ptr<utils::OdomSmoother> odom_smoother_;


    // Progress Checker Plugin
    ProgressCheckerMap progress_checkers_;
    std::vector<std::string> default_progress_checker_ids_;
    std::vector<std::string> default_progress_checker_types_;
    std::vector<std::string> progress_checker_ids_;
    std::vector<std::string> progress_checker_types_;
    std::string progress_checker_ids_concat_, current_progress_checker_;

    // Goal Checker Plugin
    GoalCheckerMap goal_checkers_;
    std::vector<std::string> default_goal_checker_ids_;
    std::vector<std::string> default_goal_checker_types_;
    std::vector<std::string> goal_checker_ids_;
    std::vector<std::string> goal_checker_types_;
    std::string goal_checker_ids_concat_, current_goal_checker_;

    // Controller Plugins
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
    commsgs::geometry_msgs::TwistStamped last_cmd_vel_{};

    bool follow_path_active_{false};
    bool recovery_active_{false};
    RecoveryMotionCommand recovery_cmd_;
    commsgs::geometry_msgs::PoseStamped recovery_start_pose_;
    double recovery_start_yaw_{0.0};
    commsgs::builtin_interfaces::Time recovery_deadline_;
    bool plugins_loaded_{false};
    bool controllers_active_{false};
    float transform_tolerance_{0.1f};

    // Dynamic parameters lock
    std::mutex dynamic_params_lock_;

private:
    /**
     * @brief Callback for speed limiting messages
     * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
     */
    void SpeedLimitCallback(
        const commsgs::planning_msgs::SpeedLimit::SharedPtr msg);

    // controller options
    proto::ControllerOptions options_;
};

}  // namespace control
}  // namespace autonomy
