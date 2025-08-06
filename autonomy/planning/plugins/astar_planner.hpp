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

#include "autonomy/planning/common/planner_interface.hpp"
namespace autonomy {
namespace planning {
namespace plugins {

class AStarPlanner : common::PlannerInterface
{
public:
    
    /**
     * Define AStarPlanner::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(AStarPlanner)

    /**
     * @brief A constructor for autonomy::planning::plugins::AStarPlanner
     * @param options Additional options to control creation of the node.
     */
    explicit AStarPlanner();

    /**
     * @brief A Destructor for autonomy::planning::plugins::AStarPlanner
     */
    virtual ~AStarPlanner();

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
        std::vector<commsgs::geometry_msgs::PoseStamped>& plan, double& cost,
        std::string& message) override;

    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    virtual bool Cancel() override;
};

}  // namespace plugins
}  // namespace planning
}  // namespace autonomy