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

#ifndef AUTONOMY_CONTROL_TEB_CONTROLLER_POSE2D_UTILS_HPP_
#define AUTONOMY_CONTROL_TEB_CONTROLLER_POSE2D_UTILS_HPP_

#include <cmath>

#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/control/controller/teb_controller/geometry_utils.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

inline Point Position(const Pose2D& pose) {
    return PointFromPose2D(pose);
}

inline Point OrientationUnitVec(const Pose2D& pose) {
    return MakePoint(std::cos(pose.theta), std::sin(pose.theta));
}

inline double AverageAngle(double angle_a, double angle_b) {
    return angle_a + autonomy::common::AngleDiff(angle_a, angle_b) * 0.5;
}

inline Pose2D AveragePose2D(const Pose2D& pose_a, const Pose2D& pose_b) {
    return {pose_a.x + pose_b.x, pose_a.y + pose_b.y,
            AverageAngle(pose_a.theta, pose_b.theta)};
}

inline double GetYawFromOrientation(
    const autonomy::commsgs::geometry_msgs::Quaternion& orientation) {
    const double siny_cosp =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cosy_cosp =
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

inline Pose2D Pose2DFromPose(const autonomy::commsgs::geometry_msgs::Pose& pose) {
    return {pose.position.x, pose.position.y,
            GetYawFromOrientation(pose.orientation)};
}

inline Pose2D Pose2DFromPoseStamped(
    const autonomy::commsgs::geometry_msgs::PoseStamped& pose) {
    return Pose2DFromPose(pose.pose);
}

inline void ToPose3D(const Pose2D& pose2d,
                     autonomy::commsgs::geometry_msgs::Pose& pose) {
    pose.position.x = pose2d.x;
    pose.position.y = pose2d.y;
    pose.position.z = 0.0;
    pose.orientation =
        map::costmap_2d::utils::OrientationAroundZAxis(pose2d.theta);
}

inline void RotateGlobal(Pose2D& pose, double angle, bool adjust_theta = true) {
    const double new_x = std::cos(angle) * pose.x - std::sin(angle) * pose.y;
    const double new_y = std::sin(angle) * pose.x + std::cos(angle) * pose.y;
    pose.x = new_x;
    pose.y = new_y;
    if (adjust_theta) {
        pose.theta = autonomy::common::NormalizeAngle(pose.theta + angle);
    }
}

inline int Sign(double value) {
    return (value > 0.0) - (value < 0.0);
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TEB_CONTROLLER_POSE2D_UTILS_HPP_
