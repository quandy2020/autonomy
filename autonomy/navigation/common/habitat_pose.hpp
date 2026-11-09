/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file habitat_pose.hpp
 * @brief ROS planar (x, y, yaw) to Habitat (x, z, yaw) used by VNT-PA RoPE.
 *
 * @details Habitat world is right-handed with +y up and yaw-0 facing -z. ROS
 * map is REP-103 with +x forward and +y left. The planar mapping is
 * habitat_x = -ros_y, habitat_z = -ros_x, yaw unchanged, so a robot at ROS
 * yaw 0 maps onto Habitat yaw 0.
 */

#pragma once

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace common
{

/**
 * @brief Convert a ROS map-frame pose into Habitat (x, z, yaw).
 *
 * @param[in] ros_pose Planar pose in metres / radians.
 */
inline Pose2D RosToHabitat(const Pose2D& ros_pose)
{
    return MakePose2D(-ros_pose.y(), -ros_pose.x(), ros_pose.theta());
}

/**
 * @brief Convert a Habitat (x, z, yaw) pose back to ROS (x, y, yaw).
 *
 * @param[in] habitat_pose Planar Habitat pose; y holds Habitat z.
 */
inline Pose2D HabitatToRos(const Pose2D& habitat_pose)
{
    return MakePose2D(-habitat_pose.y(), -habitat_pose.x(), habitat_pose.theta());
}

}  // namespace common
}  // namespace navigation
