/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/transform_utils.hpp"

#include <QQuaternion>
#include <QtMath>

namespace autoviz {
namespace display {

QVector3D rotateVector(const automsgs::msgs::geometry_msgs::Quaternion& q,
                       const QVector3D& v) {
  const float qx = static_cast<float>(q.x());
  const float qy = static_cast<float>(q.y());
  const float qz = static_cast<float>(q.z());
  const float qw = static_cast<float>(q.w());

  const float tx = 2.f * (qy * v.z() - qz * v.y());
  const float ty = 2.f * (qz * v.x() - qx * v.z());
  const float tz = 2.f * (qx * v.y() - qy * v.x());
  return QVector3D(v.x() + qw * tx + qy * tz - qz * ty,
                   v.y() + qw * ty + qz * tx - qx * tz,
                   v.z() + qw * tz + qx * ty - qy * tx);
}

QVector3D transformPoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const QVector3D& point) {
  const QVector3D rotated = rotateVector(tf.transform().rotation(), point);
  return rotated + QVector3D(static_cast<float>(tf.transform().translation().x()),
                             static_cast<float>(tf.transform().translation().y()),
                             static_cast<float>(tf.transform().translation().z()));
}

QMatrix4x4 transformToMatrix(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(static_cast<float>(tf.transform().translation().x()),
                   static_cast<float>(tf.transform().translation().y()),
                   static_cast<float>(tf.transform().translation().z()));
  const auto& q = tf.transform().rotation();
  matrix.rotate(QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                            static_cast<float>(q.y()), static_cast<float>(q.z())));
  return matrix;
}

}  // namespace display
}  // namespace autoviz
