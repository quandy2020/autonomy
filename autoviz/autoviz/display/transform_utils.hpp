/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file transform_utils.hpp
 * @brief Geometry helpers for applying @c geometry_msgs/TransformStamped.
 *
 * Converts ROS transform messages into Qt vectors / matrices for display
 * drawing code that already works in @c QVector3D / @c QMatrix4x4 space.
 *
 * @see transformPoint()
 * @see transformToMatrix()
 * @see rotateVector()
 * @see ChannelDisplay
 */

#pragma once

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

namespace autoviz {
namespace display {

/**
 * @brief Rotates @p v by quaternion @p q (no translation).
 *
 * @param q geometry_msgs quaternion.
 * @param v Vector to rotate.
 * @return Rotated vector.
 */
QVector3D rotateVector(const automsgs::msgs::geometry_msgs::Quaternion& q,
                       const QVector3D& v);

/**
 * @brief Applies a TransformStamped (rotation + translation) to a point.
 *
 * @param tf Parent←child (or fixed←frame) transform from the TF buffer.
 * @param point Point in the child / source frame.
 * @return Point in the parent / target frame.
 */
QVector3D transformPoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const QVector3D& point);

/**
 * @brief Converts a TransformStamped into a 4×4 homogeneous matrix.
 *
 * @param tf Source transform.
 * @return @c QMatrix4x4 suitable for mesh instance transforms.
 */
QMatrix4x4 transformToMatrix(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf);

}  // namespace display
}  // namespace autoviz
