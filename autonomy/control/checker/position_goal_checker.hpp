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
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
 */
class PositionGoalChecker : public common::GoalChecker
{
public:
    PositionGoalChecker();
    ~PositionGoalChecker() override = default;

    void Initialize(const std::string& plugin_name,
                    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                        costmap_wrapper) override;

    void Reset() override;

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

    /**
     * @brief Set the XY goal tolerance
     * @param tolerance New tolerance value
     */
    void SetXYGoalTolerance(double tolerance);

protected:
    double xy_goal_tolerance_;
    double xy_goal_tolerance_sq_;
    double path_length_tolerance_;
    bool stateful_;
    bool position_reached_;
    std::string plugin_name_;
};

}  // namespace checker
}  // namespace control
}  // namespace autonomy
