/*
 * Copyright 2026 The Openbot Authors
 *
 * Rigid-pose helpers over automsgs geometry_msgs::Pose (no parallel POD).
 */

#pragma once

#include <automsgs/msgs/geometry_msgs/pose.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

/** @brief Identity pose (origin, quaternion w=1). */
automsgs::msgs::geometry_msgs::Pose IdentityPose();

/**
 * @brief Compose rigid transforms: result = @p parent * @p child.
 */
automsgs::msgs::geometry_msgs::Pose ComposePoses(
    const automsgs::msgs::geometry_msgs::Pose& parent,
    const automsgs::msgs::geometry_msgs::Pose& child);

/**
 * @brief Pose from roll-pitch-yaw (rad) and translation (m).
 */
automsgs::msgs::geometry_msgs::Pose PoseFromRollPitchYawXyz(
    double roll, double pitch, double yaw, double x, double y, double z);

/**
 * @brief Pose from axis-angle rotation (translation zero).
 */
automsgs::msgs::geometry_msgs::Pose PoseFromAxisAngle(double axis_x,
                                                      double axis_y,
                                                      double axis_z,
                                                      double angle);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
