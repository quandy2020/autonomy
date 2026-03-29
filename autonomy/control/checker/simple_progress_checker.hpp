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
#include "autonomy/control/common/progress_checker_interface.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace checker {

/**
 * @class SimpleProgressChecker
 * @brief This plugin is used to check the position of the robot to make sure
 * that it is actually progressing towards a goal.
 */

class SimpleProgressChecker : public common::ProgressChecker
{
public:
    void Initialize(const std::string& plugin_name) override;
    bool Check(commsgs::geometry_msgs::PoseStamped& current_pose) override;
    void Reset() override;

protected:
    /**
     * @brief Calculates robots movement from baseline pose
     * @param pose Current pose of the robot
     * @return true, if movement is greater than radius_, or false
     */
    bool IsRobotMovedEnough(const commsgs::geometry_msgs::Pose2D& pose);
    /**
     * @brief Resets baseline pose with the current pose of the robot
     * @param pose Current pose of the robot
     */
    void ResetBaselinePose(const commsgs::geometry_msgs::Pose2D& pose);

    static double PoseDistance(const commsgs::geometry_msgs::Pose2D&,
                               const commsgs::geometry_msgs::Pose2D&);

    double radius_;
    commsgs::geometry_msgs::Pose2D baseline_pose_;
    // commsgs::time::Time baseline_time_;

    bool baseline_pose_set_{false};
    std::string plugin_name_;
};

}  // namespace checker
}  // namespace control
}  // namespace autonomy