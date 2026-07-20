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
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"

namespace autonomy {
namespace control {
namespace checker {

/**
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
 *
 * This class can be stateful if the stateful parameter is set to true (which it
 * is by default). This means that the goal checker will not check if the xy
 * position matches again once it is found to be true.
 */
class SimpleGoalChecker : public common::GoalChecker
{
public:
    SimpleGoalChecker();
    // Standard GoalChecker Interface
    void Initialize(const std::string& plugin_name,
                    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                        costmap_wraper) override;

    void Reset() override;

    void SetTolerances(double xy_tolerance, double yaw_tolerance,
                       bool stateful = true,
                       double path_length_tolerance = 1.0,
                       double xy_goal_tolerance_buffer = 0.0);

    bool IsGoalReached(const commsgs::geometry_msgs::Pose& query_pose,
                       const commsgs::geometry_msgs::Pose& goal_pose,
                       const commsgs::geometry_msgs::Twist& velocity) override;

    bool IsGoalXYReached(
        const commsgs::geometry_msgs::Pose& query_pose,
        const commsgs::geometry_msgs::Pose& goal_pose,
        const commsgs::geometry_msgs::Twist& velocity,
        const commsgs::planning_msgs::Path& transformed_global_plan) override;

    bool GetTolerances(commsgs::geometry_msgs::Pose& pose_tolerance,
                       commsgs::geometry_msgs::Twist& vel_tolerance) override;

protected:
    double xy_goal_tolerance_, yaw_goal_tolerance_;
    double xy_goal_tolerance_buffer_;
    double path_length_tolerance_;
    bool stateful_, check_xy_;
    // Cached squared xy_goal_tolerance_
    double xy_goal_tolerance_sq_;
    double xy_goal_tolerance_reset_sq_;
    std::string plugin_name_;
};

}  // namespace checker
}  // namespace control
}  // namespace autonomy