/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QImage>
#include <QMatrix4x4>
#include <QPointF>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autoviz {
namespace image {

struct CameraIntrinsics {
  int width = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::vector<double> distortion;
  std::string distortion_model;
  bool valid = false;
};

CameraIntrinsics intrinsicsFromCameraInfo(
    const automsgs::msgs::sensor_msgs::CameraInfo& info);

QMatrix4x4 fixedToOpticalMatrix(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& fixed_to_camera);

std::optional<QPointF> projectOpticalPointToPixel(const CameraIntrinsics& intrinsics,
                                                  const QVector3D& optical_point);

std::optional<QPointF> projectFixedPointToPixel(
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const QVector3D& fixed_point);

QImage undistortImage(const QImage& source, const CameraIntrinsics& intrinsics);

}  // namespace image
}  // namespace autoviz
