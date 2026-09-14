/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 */
#include "autonomy/common/transform/rigid_transform.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace autonomy {
namespace common {
namespace transform {

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return yaw_angle * pitch_angle * roll_angle;
}

}  // namespace transform
}  // namespace common
}  // namespace autonomy
