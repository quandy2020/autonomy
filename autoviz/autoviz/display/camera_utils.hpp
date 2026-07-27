/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autoviz {
namespace display {

struct CameraFarPlane {
  QVector3D top_left;
  QVector3D top_right;
  QVector3D bottom_right;
  QVector3D bottom_left;
  bool valid = false;
};

/** Camera frustum segment pairs in optical frame (ROS: x right, y down, z forward). */
std::vector<std::pair<QVector3D, QVector3D>> buildCameraFrustumSegments(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float near_distance, float far_distance);

/** Far-plane rectangle corners in optical frame for texture projection. */
CameraFarPlane buildCameraFarPlane(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float far_distance);

QMatrix4x4 opticalToWorld(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& camera_to_fixed);

}  // namespace display
}  // namespace autoviz
