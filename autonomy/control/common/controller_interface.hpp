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

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
namespace transform {
class Buffer;
}  // namespace transform
}  // namespace autonomy

namespace autonomy {
namespace control {
namespace common {

/**
 * @class ControllerInterface
 * @brief Virtual base class for local controller plugins.
 *
 * Plugins are constructed with options, name, TF buffer, and costmap wrapper.
 * Provide a default constructor for Autolink plugin registration; use the
 * protected parameterized constructor for runtime instantiation.
 */
class ControllerInterface
{
public:
    /**
     * @brief Type definition for transform buffer.
     */
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * @brief Define ControllerInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ControllerInterface)

    /**
     * @brief Destructor for ControllerInterface
     */
    virtual ~ControllerInterface() = default;

    /** 
     * @brief Controller options from configuration.
     * @return Controller options
     */
    const proto::ControllerOptions& GetOptions() const { return options_; }

    /**
     * @brief Plugin instance name (e.g. from controller_plugins).
     * @return Plugin instance name
     */
    const std::string& GetName() const { return name_; }

    /** 
     * @brief Transform buffer used for pose lookups.
     * @return Transform buffer pointer
     */
    TfBuffer* GetTfBuffer() const { return tf_buffer_.get(); }

    /** 
     * @brief Local costmap wrapper (may be null until set by server).
     * @return Local costmap wrapper pointer
     */
    map::costmap_2d::Costmap2DWrapper* GetCostmap() const {
        return costmap_wrapper_.get();
    }

    /**
     * @brief Given the current position, orientation, and velocity of the
     * robot, compute velocity commands to send to the base.
     * @param pose The current pose of the robot.
     * @param velocity The current velocity of the robot.
     * @param cmd_vel Will be filled with the velocity command to be passed to
     * the robot base. The frame id will set to the robot frame id by default,
     * but can be added inside the implementation.
     * @param goal_checker Goal checker used to evaluate terminal conditions
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
     * @brief Reset controller state when navigation ends.
     */
    virtual void Reset() {}

protected:
    /** 
     * @brief Default constructor for plugin registration only.
     */
    ControllerInterface() = default;

    /**
     * @brief Construct and initialize a controller plugin.
     * @param options Controller options from configuration
     * @param name Plugin instance name
     * @param tf_buffer Transform buffer for pose lookups
     * @param costmap_wrapper Local costmap wrapper
     */
    ControllerInterface(const proto::ControllerOptions& options,
                        std::string name, std::shared_ptr<TfBuffer> tf_buffer,
                        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

    // Controller options from configuration.
    proto::ControllerOptions options_;

    // Plugin instance name.
    std::string name_;

    // Transform buffer used for pose lookups.
    std::shared_ptr<TfBuffer> tf_buffer_;

    // Local costmap wrapper (may be null until set by server).
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
};

}  // namespace common
}  // namespace control
}  // namespace autonomy
