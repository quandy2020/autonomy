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

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace common {

class ControllerInterface
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define ControllerInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ControllerInterface)

    /**
     * @brief A Destructor for autonomy::control::common::ControllerInterface
     */
    virtual ~ControllerInterface() = default;

    /**
     * @param  parent pointer to user's node
     * @param  costmap_ros A pointer to the costmap
     */
    virtual void Configure(
        std::string name, std::shared_ptr<TfBuffer>,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper>) = 0;

    /**
     * @brief Method to cleanup resources.
     */
    virtual void Cleanup() = 0;

    /**
     * @brief Method to active planner and any threads involved in execution.
     */
    virtual void Activate() = 0;

    /**
     * @brief Method to deactive planner and any threads involved in execution.
     */
    virtual void Deactivate() = 0;

    /**
     * @brief Given the current position, orientation, and velocity of the
     * robot, compute velocity commands to send to the base.
     * @param pose The current pose of the robot.
     * @param velocity The current velocity of the robot.
     * @param cmd_vel Will be filled with the velocity command to be passed to
     * the robot base. The frame id will set to the robot frame id by default,
     * but can be added inside the implementation.
     * @param message Optional more detailed outcome as a string
     * @return Result code from ControllerResultCode enum (see
     * autonomy.control.proto.ControllerResultCode) Return values correspond to
     * ControllerResultCode enum values:
     *         - CONTROLLER_RESULT_SUCCESS (0): Success
     *         - 1..9: Reserved for plugin specific non-error results
     *         - CONTROLLER_RESULT_FAILURE (100) and higher: Standard error
     * codes
     *         - 121..149: Reserved for plugin specific errors
     */
    virtual uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) = 0;

    /**
     * @brief Check if the goal pose has been achieved by the local planner
     * @param angle_tolerance The angle tolerance in which the current pose will
     * be partly accepted as reached goal
     * @param dist_tolerance The distance tolerance in which the current pose
     * will be partly accepted as reached goal
     * @return True if achieved, false otherwise
     */
    virtual bool IsGoalReached(double dist_tolerance,
                               double angle_tolerance) = 0;

    /**
     * @brief Set the plan that the local planner is following
     * @param plan The plan to pass to the local planner
     */
    virtual void SetPlan(const commsgs::planning_msgs::Path& plan) = 0;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    virtual void SetSpeedLimit(const double& speed_limit,
                               const bool& percentage) = 0;

    /**
     * @brief Reset the state of the controller if necessary after task is
     * exited
     */
    virtual void Reset() {}
};

}  // namespace common
}  // namespace control
}  // namespace autonomy