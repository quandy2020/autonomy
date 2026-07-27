/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

namespace autoviz {
namespace display {

QVector3D rotateVector(const automsgs::msgs::geometry_msgs::Quaternion& q,
                       const QVector3D& v);

QVector3D transformPoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const QVector3D& point);

QMatrix4x4 transformToMatrix(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf);

}  // namespace display
}  // namespace autoviz
