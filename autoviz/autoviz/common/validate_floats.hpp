/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cmath>

#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>

namespace autoviz {
namespace common {

/** rviz_common::validateFloats — reject NaN/Inf in display inputs. */
inline bool validateFloats(float val) {
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(double val) {
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(const QVector3D& vec) {
  return validateFloats(vec.x()) && validateFloats(vec.y()) &&
         validateFloats(vec.z());
}

inline bool validateFloats(const QQuaternion& quat) {
  return validateFloats(quat.scalar()) && validateFloats(quat.x()) &&
         validateFloats(quat.y()) && validateFloats(quat.z());
}

inline bool validateFloats(
    const automsgs::msgs::geometry_msgs::Vector3& msg) {
  return validateFloats(msg.x()) && validateFloats(msg.y()) &&
         validateFloats(msg.z());
}

inline bool validateFloats(
    const automsgs::msgs::geometry_msgs::Quaternion& msg) {
  return validateFloats(msg.x()) && validateFloats(msg.y()) &&
         validateFloats(msg.z()) && validateFloats(msg.w());
}

}  // namespace common
}  // namespace autoviz
