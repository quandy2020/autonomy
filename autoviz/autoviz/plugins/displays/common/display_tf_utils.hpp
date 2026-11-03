/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace viz {

inline QQuaternion QuatFromProto(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  return QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                     static_cast<float>(q.y()), static_cast<float>(q.z()));
}

inline QMatrix4x4 PoseToMatrix(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(static_cast<float>(pose.position().x()),
                   static_cast<float>(pose.position().y()),
                   static_cast<float>(pose.position().z()));
  matrix.rotate(QuatFromProto(pose.orientation()));
  return matrix;
}

inline bool LookupFixedFrameTransform(
    common::DisplayContext* context, const std::string& frame_id,
    automsgs::msgs::geometry_msgs::TransformStamped* tf_out) {
  if (context == nullptr || tf_out == nullptr) {
    return false;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = frame_id.empty() ? context->fixed_frame : frame_id;
  try {
    *tf_out = context->tf_buffer->lookupTransform(context->fixed_frame, frame,
                                                  zero_time);
    return true;
  } catch (...) {
    return false;
  }
}

inline QVector3D TransformLocalPoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf, bool have_tf,
    const QVector3D& local) {
  return have_tf ? transformPoint(tf, local) : local;
}

inline QVector3D TransformPosePoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf, bool have_tf,
    const automsgs::msgs::geometry_msgs::Pose& pose, const QVector3D& local) {
  const QMatrix4x4 pose_matrix = PoseToMatrix(pose);
  const QVector3D in_frame =
      (pose_matrix * QVector4D(local, 1.f)).toVector3D();
  return TransformLocalPoint(tf, have_tf, in_frame);
}

}  // namespace viz
}  // namespace display
}  // namespace autoviz
