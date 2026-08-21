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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include "autonomy/control/common/progress_checker_interface.hpp"

namespace autonomy {
namespace control {
namespace checker {

/**
 * @class SimpleProgressChecker
 * @brief Ensures the robot advances at least required_movement_radius within
 *        movement_time_allowance (Nav2-compatible semantics).
 */
class SimpleProgressChecker : public common::ProgressChecker
{
public:
    void Initialize(const std::string& plugin_name) override;
    bool Check(automsgs::msgs::geometry_msgs::PoseStamped& current_pose) override;
    void Reset() override;

    void SetRequiredMovementRadius(double radius);
    void SetMovementTimeAllowance(double seconds);

protected:
    /**
     * @brief Calculates robots movement from baseline pose
     * @param pose Current pose of the robot
     * @return true, if movement is greater than radius_, or false
     */
    bool IsRobotMovedEnough(const automsgs::msgs::geometry_msgs::Pose2D& pose);
    /**
     * @brief Resets baseline pose with the current pose of the robot
     * @param pose Current pose of the robot
     */
    void ResetBaselinePose(const automsgs::msgs::geometry_msgs::Pose2D& pose);

    static double PoseDistance(const automsgs::msgs::geometry_msgs::Pose2D&,
                               const automsgs::msgs::geometry_msgs::Pose2D&);

    double radius_{0.5};
    double time_allowance_sec_{10.0};
    automsgs::msgs::geometry_msgs::Pose2D baseline_pose_;
    std::chrono::steady_clock::time_point baseline_time_{};

    bool baseline_pose_set_{false};
    std::string plugin_name_;
};

}  // namespace checker
}  // namespace control
}  // namespace autonomy
