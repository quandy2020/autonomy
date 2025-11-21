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

#include "autonomy/planning/proto/planning_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace planning {
namespace common {

class PlannerInterface
{
public:

    /**
     * @details The transform buffer is used to transform poses between frames.
     */
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define PlannerInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerInterface)

    /**
     * @brief A constructor for autonomy::planning::common::PlannerInterface
     * @param options Additional options to control creation of the node.
     */
    PlannerInterface() = default;

    /**
     * @brief A Destructor for autonomy::planning::common::PlannerInterface
     */
    virtual ~PlannerInterface() = default;

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
     *        in x and y before failing
     * @param plan The plan... filled by the planner
     * @param cost The cost for the the plan
     * @param message Optional more detailed outcome as a string
     * @return Result code as described on GetPath action result:
     *         SUCCESS         = 0
     *         1..9 are reserved as plugin specific non-error results
     *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
     *         CANCELED        = 51
     *         INVALID_START   = 52
     *         INVALID_GOAL    = 53
     *         BLOCKED_START   = 54
     *         BLOCKED_GOAL    = 55
     *         NO_PATH_FOUND   = 56
     *         PAT_EXCEEDED    = 57
     *         EMPTY_PATH      = 58
     *         TF_ERROR        = 59
     *         NOT_INITIALIZED = 60
     *         INVALID_PLUGIN  = 61
     *         INTERNAL_ERROR  = 62
     *         71..99 are reserved as plugin specific errors
     */
    virtual uint32 MakePlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal, double tolerance,
        commsgs::planning_msgs::Path& plan, double& cost,
        std::string& message) = 0;

    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    virtual bool Cancel() = 0;

    /**
     * @brief Configures the planner with the given options
     * @param options The options to configure the planner with
     * @return True if the planner was successfully configured, false otherwise
     */
    virtual bool Configure(const proto::PlannerOptions& options, const std::string& name, 
        TfBuffer* tf_buffer, map::costmap_2d::Costmap2DWrapper* costmap_wrapper) = 0;

    /**
     * @brief Activates the planner
     * @return True if the planner was successfully activated, false otherwise
     */
    virtual bool Activate() = 0;

    /**
     * @brief Deactivates the planner
     * @return True if the planner was successfully deactivated, false otherwise
     */
    virtual bool Deactivate() = 0;

    /**
     * @brief Cleans up the planner
     * @return True if the planner was successfully cleaned up, false otherwise
     */
    virtual bool Cleanup() = 0;

    /**
     * @brief Shutdown the planner
     * @return True if the planner was successfully shut down, false otherwise
     */
    virtual bool Shutdown() = 0;

};

proto::PlannerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace common
}  // namespace planning
}  // namespace autonomy