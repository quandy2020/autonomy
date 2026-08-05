/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_calibration_utils.hpp"

#include <algorithm>
#include <cmath>

#include "autoviz/display/camera_utils.hpp"

namespace autoviz {
namespace image {
namespace {

QVector3D DistortNormalizedPoint(double x, double y,
                                 const std::vector<double>& d) {
  const double k1 = d.size() > 0 ? d[0] : 0.0;
  const double k2 = d.size() > 1 ? d[1] : 0.0;
  const double p1 = d.size() > 2 ? d[2] : 0.0;
  const double p2 = d.size() > 3 ? d[3] : 0.0;
  const double k3 = d.size() > 4 ? d[4] : 0.0;
  const double r2 = x * x + y * y;
  const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  const double x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  return QVector3D(static_cast<float>(x_distorted), static_cast<float>(y_distorted),
                   1.0f);
}

std::optional<QPointF> UndistortPixel(const CameraIntrinsics& intrinsics, double u,
                                      double v) {
  if (!intrinsics.valid || intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return std::nullopt;
  }
  double x = (u - intrinsics.cx) / intrinsics.fx;
  double y = (v - intrinsics.cy) / intrinsics.fy;
  if (intrinsics.distortion.empty()) {
    return QPointF(u, v);
  }
  for (int iter = 0; iter < 8; ++iter) {
    const QVector3D distorted = DistortNormalizedPoint(x, y, intrinsics.distortion);
    const double dx = distorted.x() - x;
    const double dy = distorted.y() - y;
    x -= dx;
    y -= dy;
  }
  return QPointF(intrinsics.fx * x + intrinsics.cx, intrinsics.fy * y + intrinsics.cy);
}

}  // namespace

CameraIntrinsics intrinsicsFromCameraInfo(
    const automsgs::msgs::sensor_msgs::CameraInfo& info) {
  CameraIntrinsics intrinsics;
  intrinsics.width = static_cast<int>(info.width());
  intrinsics.height = static_cast<int>(info.height());
  if (info.k_size() >= 9) {
    intrinsics.fx = info.k(0);
    intrinsics.fy = info.k(4);
    intrinsics.cx = info.k(2);
    intrinsics.cy = info.k(5);
  }
  intrinsics.distortion_model = info.distortion_model();
  intrinsics.distortion.assign(info.d().begin(), info.d().end());
  intrinsics.valid = intrinsics.width > 0 && intrinsics.height > 0 &&
                     intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
  return intrinsics;
}

QMatrix4x4 fixedToOpticalMatrix(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& camera_to_fixed) {
  Q_UNUSED(info);
  const QMatrix4x4 optical_to_camera(
      0.0f, 0.0f, 1.0f, 0.0f,
      -1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, -1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f);
  const QMatrix4x4 fixed_to_camera = camera_to_fixed.inverted();
  return optical_to_camera.inverted() * fixed_to_camera;
}

std::optional<QPointF> projectOpticalPointToPixel(
    const CameraIntrinsics& intrinsics, const QVector3D& optical_point) {
  if (!intrinsics.valid || optical_point.z() <= 1e-6f) {
    return std::nullopt;
  }
  const double u = intrinsics.fx * (optical_point.x() / optical_point.z()) +
                   intrinsics.cx;
  const double v = intrinsics.fy * (optical_point.y() / optical_point.z()) +
                   intrinsics.cy;
  if (u < 0.0 || v < 0.0 || u >= intrinsics.width || v >= intrinsics.height) {
    return std::nullopt;
  }
  return QPointF(u, v);
}

std::optional<QPointF> projectFixedPointToPixel(
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const QVector3D& fixed_point) {
  const QVector3D optical = fixed_to_optical.map(fixed_point);
  return projectOpticalPointToPixel(intrinsics, optical);
}

QImage undistortImage(const QImage& source, const CameraIntrinsics& intrinsics) {
  if (source.isNull() || !intrinsics.valid || intrinsics.distortion.empty()) {
    return source;
  }
  QImage rgb = source.format() == QImage::Format_RGB888
                   ? source
                   : source.convertToFormat(QImage::Format_RGB888);
  QImage output(rgb.size(), QImage::Format_RGB888);
  output.fill(Qt::black);
  for (int v = 0; v < rgb.height(); ++v) {
    for (int u = 0; u < rgb.width(); ++u) {
      const double x = (u + 0.5 - intrinsics.cx) / intrinsics.fx;
      const double y = (v + 0.5 - intrinsics.cy) / intrinsics.fy;
      const QVector3D distorted =
          DistortNormalizedPoint(x, y, intrinsics.distortion);
      const int sx = std::clamp(
          static_cast<int>(distorted.x() * intrinsics.fx + intrinsics.cx), 0,
          rgb.width() - 1);
      const int sy = std::clamp(
          static_cast<int>(distorted.y() * intrinsics.fy + intrinsics.cy), 0,
          rgb.height() - 1);
      output.setPixel(u, v, rgb.pixel(sx, sy));
    }
  }
  return output;
}

}  // namespace image
}  // namespace autoviz
