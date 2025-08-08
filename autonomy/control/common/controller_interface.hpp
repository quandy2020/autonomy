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

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace control {
namespace common {

class ControllerInterface
{
public:
    /**
     * Define ControllerInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ControllerInterface)

    /**
     * @brief A constructor for autonomy::control::common::ControllerInterface
     * @param options Additional options to control creation of the node.
     */
    explicit ControllerInterface();

    /**
     * @brief A Destructor for autonomy::control::common::ControllerInterface
     */
    virtual ~ControllerInterface();

    /**
       * @brief Given the current position, orientation, and velocity of the robot,
    * compute velocity commands to send to the base.
    * @param pose The current pose of the robot.
    * @param velocity The current velocity of the robot.
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base. The frame id will set
    * to the robot frame id by default, but can be added inside the implementation.
    * @param message Optional more detailed outcome as a string
    * @return Result code as described on ExePath action result:
    *         SUCCESS           = 0
    *         1..9 are reserved as plugin specific non-error results
    *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
    *         CANCELED          = 101
    *         NO_VALID_CMD      = 102
    *         PAT_EXCEEDED      = 103
    *         COLLISION         = 104
    *         OSCILLATION       = 105
    *         ROBOT_STUCK       = 106
    *         MISSED_GOAL       = 107
    *         MISSED_PATH       = 108
    *         BLOCKED_GOAL      = 109
    *         BLOCKED_PATH      = 110
    *         INVALID_PATH      = 111
    *         TF_ERROR          = 112
    *         NOT_INITIALIZED   = 113
    *         INVALID_PLUGIN    = 114
    *         INTERNAL_ERROR    = 115
    *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
    *         MAP_ERROR         = 117  # The map is not running properly
    *         STOPPED           = 118  # The controller execution has been stopped rigorously
    *         121..149 are reserved as plugin specific errors
    */
    virtual uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel, 
        std::string& message) = 0;

    /**
     * @brief Check if the goal pose has been achieved by the local planner
     * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
     * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
     * @return True if achieved, false otherwise
     */
    virtual bool IsGoalReached(double dist_tolerance, double angle_tolerance) = 0;

    /**
     * @brief Set the plan that the local planner is following
     * @param plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    virtual bool SetPlan(const std::vector<commsgs::geometry_msgs::PoseStamped>& plan) = 0;

    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    virtual bool Cancel() = 0;
};

}  // namespace common
}  // namespace control
}  // namespace autonomy