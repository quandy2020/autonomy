/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file validate_floats.hpp
 * @brief Reject NaN/Inf in display and geometry inputs (RViz validateFloats).
 *
 * Displays should call these before applying message fields to the scene to
 * avoid propagating non-finite values into renderers and TF math.
 *
 * @note Overloads cover scalars, Qt math types, and Automsgs geometry protos.
 */

#pragma once

#include <cmath>

#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>

namespace autoviz {
namespace common {

/**
 * @brief Returns @c true if @p val is finite (not NaN or Inf).
 * @param val Float to test.
 * @return Whether the value is usable.
 */
inline bool validateFloats(float val) {
  return !(std::isnan(val) || std::isinf(val));
}

/**
 * @brief Returns @c true if @p val is finite (not NaN or Inf).
 * @param val Double to test.
 * @return Whether the value is usable.
 */
inline bool validateFloats(double val) {
  return !(std::isnan(val) || std::isinf(val));
}

/**
 * @brief Validates all components of a @c QVector3D.
 * @param vec Vector to test.
 * @return @c true if x, y, and z are all finite.
 */
inline bool validateFloats(const QVector3D& vec) {
  return validateFloats(vec.x()) && validateFloats(vec.y()) &&
         validateFloats(vec.z());
}

/**
 * @brief Validates all components of a @c QQuaternion (scalar + xyz).
 * @param quat Quaternion to test.
 * @return @c true if all components are finite.
 */
inline bool validateFloats(const QQuaternion& quat) {
  return validateFloats(quat.scalar()) && validateFloats(quat.x()) &&
         validateFloats(quat.y()) && validateFloats(quat.z());
}

/**
 * @brief Validates an Automsgs @c geometry_msgs::Vector3.
 * @param msg Protobuf vector message.
 * @return @c true if x, y, and z are finite.
 */
inline bool validateFloats(
    const automsgs::msgs::geometry_msgs::Vector3& msg) {
  return validateFloats(msg.x()) && validateFloats(msg.y()) &&
         validateFloats(msg.z());
}

/**
 * @brief Validates an Automsgs @c geometry_msgs::Quaternion.
 * @param msg Protobuf quaternion message.
 * @return @c true if x, y, z, and w are finite.
 */
inline bool validateFloats(
    const automsgs::msgs::geometry_msgs::Quaternion& msg) {
  return validateFloats(msg.x()) && validateFloats(msg.y()) &&
         validateFloats(msg.z()) && validateFloats(msg.w());
}

}  // namespace common
}  // namespace autoviz
