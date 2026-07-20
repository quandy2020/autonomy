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
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

namespace autonomy {
namespace control {
namespace common {

/**
 * @class GoalChecker
 * @brief Function-object for checking whether a goal has been reached
 *
 * This class defines the plugin interface for determining whether you have
 * reached the goal state. This primarily consists of checking the relative
 * positions of two poses (which are presumed to be in the same frame). It can
 * also check the velocity, as some applications require that robot be stopped
 * to be considered as having reached the goal.
 */
class GoalChecker
{
public:
    /**
     * Define GoalChecker::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(GoalChecker)

    /**
     * @brief A Destructor for GoalChecker
     */
    virtual ~GoalChecker() = default;

    /**
     * @brief A constructor for control::common::GoalChecker
     * @param parent Node pointer for grabbing parameters
     */
    virtual void Initialize(
        const std::string& plugin_name,
        const std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
            costmap_wraper) = 0;

    virtual void Reset() = 0;

    /**
     * @brief Check whether the goal should be considered reached
     * @param query_pose The pose to check
     * @param goal_pose The pose to check against
     * @param velocity The robot's current velocity
     * @return True if goal is reached
     */
    virtual bool IsGoalReached(
        const commsgs::geometry_msgs::Pose& query_pose,
        const commsgs::geometry_msgs::Pose& goal_pose,
        const commsgs::geometry_msgs::Twist& velocity) = 0;

    /**
     * @brief Check whether XY position goal has been reached (yaw ignored).
     * @param transformed_global_plan Remaining global plan in global frame; used
     * to gate early trigger on closed loops when path length exceeds tolerance.
     */
    virtual bool IsGoalXYReached(
        const commsgs::geometry_msgs::Pose& query_pose,
        const commsgs::geometry_msgs::Pose& goal_pose,
        const commsgs::geometry_msgs::Twist& velocity,
        const commsgs::planning_msgs::Path& transformed_global_plan);

    /**
     * @brief Get the maximum possible tolerances used for goal checking in the
     * major types. Any field without a valid entry is replaced with
     * std::numeric_limits<double>::lowest() to indicate that it is not
     * measured. For tolerance across multiple entries (e.x. XY tolerances),
     * both fields will contain this value since it is the maximum tolerance
     * that each independent field could be assuming the other has no error
     * (e.x. X and Y).
     * @param pose_tolerance The tolerance used for checking in Pose fields
     * @param vel_tolerance The tolerance used for checking velocity fields
     * @return True if the tolerances are valid to use
     */
    virtual bool GetTolerances(
        commsgs::geometry_msgs::Pose& pose_tolerance,
        commsgs::geometry_msgs::Twist& vel_tolerance) = 0;
};

inline bool GoalChecker::IsGoalXYReached(
    const commsgs::geometry_msgs::Pose& query_pose,
    const commsgs::geometry_msgs::Pose& goal_pose,
    const commsgs::geometry_msgs::Twist& velocity,
    const commsgs::planning_msgs::Path& transformed_global_plan) {
    (void)velocity;
    (void)transformed_global_plan;
    commsgs::geometry_msgs::Pose pose_tolerance;
    commsgs::geometry_msgs::Twist vel_tolerance;
    if (!GetTolerances(pose_tolerance, vel_tolerance)) {
        return false;
    }
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    const double tol = pose_tolerance.position.x;
    return dx * dx + dy * dy <= tol * tol;
}

}  // namespace common
}  // namespace control
}  // namespace autonomy