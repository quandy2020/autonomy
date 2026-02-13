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
#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace checker {

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that checks the position difference and velocity
 */
class StoppedGoalChecker : public SimpleGoalChecker
{
public:
    StoppedGoalChecker();

    // Standard GoalChecker Interface
    void Initialize(const std::string& plugin_name,
                    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) override;

    bool IsGoalReached(const commsgs::geometry_msgs::Pose& query_pose, const commsgs::geometry_msgs::Pose& goal_pose,
                       const commsgs::geometry_msgs::Twist& velocity) override;

    bool GetTolerances(commsgs::geometry_msgs::Pose& pose_tolerance,
                       commsgs::geometry_msgs::Twist& vel_tolerance) override;

protected:
    double rot_stopped_velocity_, trans_stopped_velocity_;
    std::string plugin_name_;
};

}  // namespace checker
}  // namespace control
}  // namespace autonomy