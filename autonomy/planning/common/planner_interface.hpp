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

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace planning {
namespace common {

class GlobalPlanner
{
public:
    /**
     * @details The transform buffer is used to transform poses between frames.
     */
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define GlobalPlanner::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(GlobalPlanner)

    /**
     * @brief A constructor for autonomy::planning::common::GlobalPlanner
     * @param options Additional options to control creation of the node.
     */
    GlobalPlanner() = default;

    /**
     * @brief A Destructor for autonomy::planning::common::GlobalPlanner
     */
    virtual ~GlobalPlanner() = default;

    /**
     * @brief Configures the planner with the given options
     * @param options The options to configure the planner with
     * @param name The name of the planner
     * @param tf_buffer The transform buffer
     * @param costmap Pointer to the costmap (optional, can be nullptr if set
     * later)
     * @return True if the planner was successfully configured, false otherwise
     */
    virtual bool Configure(const proto::PlannerOptions& options,
                           const std::string& name,
                           std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                               costmap = nullptr) = 0;

    /**
     * @brief Method to cleanup resources used on shutdown.
     */
    virtual void Cleanup() = 0;

    /**
     * @brief Method to active planner and any threads involved in execution.
     */
    virtual void Activate() = 0;

    /**
     * @brief Method to deactivate planner and any threads involved in
     * execution.
     */
    virtual void Deactivate() = 0;

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param tolerance If the goal is obstructed, how many meters the planner
     * can relax the constraint in x and y before failing
     * @param plan The plan... filled by the planner
     * @param cost The cost for the the plan
     * @param message Optional more detailed outcome as a string
     * @return Result code from proto::PlannerResultCode enum:
     *         - PLANNER_SUCCESS (0): Success
     *         - 1..9: Reserved for plugin specific non-error results
     *         - PLANNER_FAILURE (50): Unspecified failure
     *         - PLANNER_CANCELED (51): Operation was canceled
     *         - PLANNER_INVALID_START (52): Invalid start pose
     *         - PLANNER_INVALID_GOAL (53): Invalid goal pose
     *         - PLANNER_BLOCKED_START (54): Start pose is blocked
     *         - PLANNER_BLOCKED_GOAL (55): Goal pose is blocked
     *         - PLANNER_NO_PATH_FOUND (56): No valid path found
     *         - PLANNER_PAT_EXCEEDED (57): Path computation time exceeded
     *         - PLANNER_EMPTY_PATH (58): Generated path is empty
     *         - PLANNER_TF_ERROR (59): Transform error
     *         - PLANNER_NOT_INITIALIZED (60): Planner not initialized
     *         - PLANNER_INVALID_PLUGIN (61): Invalid plugin
     *         - PLANNER_INTERNAL_ERROR (62): Internal error
     *         - 71..99: Reserved for plugin specific errors
     */
    virtual uint32 CreatePlan(const commsgs::geometry_msgs::PoseStamped& start,
                              const commsgs::geometry_msgs::PoseStamped& goal,
                              commsgs::planning_msgs::Path& plan,
                              std::function<bool()> cancel_checker) = 0;
};

}  // namespace common
}  // namespace planning
}  // namespace autonomy