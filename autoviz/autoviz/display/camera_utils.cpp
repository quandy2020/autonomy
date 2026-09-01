/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/camera_utils.hpp"

#include <algorithm>

namespace autoviz {
namespace display {
namespace {

QVector3D FrustumCorner(float x, float y, float depth) {
  return QVector3D(x, y, depth);
}

void AppendRect(std::vector<std::pair<QVector3D, QVector3D>>* segments,
                const QVector3D& a, const QVector3D& b, const QVector3D& c,
                const QVector3D& d) {
  segments->push_back({a, b});
  segments->push_back({b, c});
  segments->push_back({c, d});
  segments->push_back({d, a});
}

void AppendPlaneCorners(std::vector<std::pair<QVector3D, QVector3D>>* segments,
                        const automsgs::msgs::sensor_msgs::CameraInfo&
                            info,
                        float depth) {
  if (info.k_size() < 9 || depth <= 0.f) {
    return;
  }
  const float fx = static_cast<float>(info.k(0));
  const float fy = static_cast<float>(info.k(4));
  const float cx = static_cast<float>(info.k(2));
  const float cy = static_cast<float>(info.k(5));
  const float width = static_cast<float>(info.width());
  const float height = static_cast<float>(info.height());
  if (fx <= 0.f || fy <= 0.f || width <= 0.f || height <= 0.f) {
    return;
  }

  const QVector3D tl =
      FrustumCorner(-cx * depth / fx, -cy * depth / fy, depth);
  const QVector3D tr =
      FrustumCorner((width - cx) * depth / fx, -cy * depth / fy, depth);
  const QVector3D br = FrustumCorner((width - cx) * depth / fx,
                                     (height - cy) * depth / fy, depth);
  const QVector3D bl =
      FrustumCorner(-cx * depth / fx, (height - cy) * depth / fy, depth);
  AppendRect(segments, tl, tr, br, bl);
}

}  // namespace

std::vector<std::pair<QVector3D, QVector3D>> buildCameraFrustumSegments(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float near_distance, float far_distance) {
  std::vector<std::pair<QVector3D, QVector3D>> segments;
  const float near_d = std::max(0.05f, near_distance);
  const float far_d = std::max(near_d + 0.05f, far_distance);
  AppendPlaneCorners(&segments, info, near_d);
  AppendPlaneCorners(&segments, info, far_d);

  if (info.k_size() >= 9) {
    const float fx = static_cast<float>(info.k(0));
    const float fy = static_cast<float>(info.k(4));
    const float cx = static_cast<float>(info.k(2));
    const float cy = static_cast<float>(info.k(5));
    const float width = static_cast<float>(info.width());
    const float height = static_cast<float>(info.height());
    const auto corner_at = [&](float depth, float px, float py) {
      return FrustumCorner((px - cx) * depth / fx, (py - cy) * depth / fy,
                           depth);
    };
    const QVector3D near_tl = corner_at(near_d, 0.f, 0.f);
    const QVector3D near_tr = corner_at(near_d, width, 0.f);
    const QVector3D near_br = corner_at(near_d, width, height);
    const QVector3D near_bl = corner_at(near_d, 0.f, height);
    const QVector3D far_tl = corner_at(far_d, 0.f, 0.f);
    const QVector3D far_tr = corner_at(far_d, width, 0.f);
    const QVector3D far_br = corner_at(far_d, width, height);
    const QVector3D far_bl = corner_at(far_d, 0.f, height);
    segments.push_back({near_tl, far_tl});
    segments.push_back({near_tr, far_tr});
    segments.push_back({near_br, far_br});
    segments.push_back({near_bl, far_bl});
  }
  return segments;
}

CameraFarPlane buildCameraFarPlane(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float far_distance) {
  CameraFarPlane plane;
  const float far_d = std::max(0.05f, far_distance);
  if (info.k_size() < 9 || far_d <= 0.f) {
    return plane;
  }
  const float fx = static_cast<float>(info.k(0));
  const float fy = static_cast<float>(info.k(4));
  const float cx = static_cast<float>(info.k(2));
  const float cy = static_cast<float>(info.k(5));
  const float width = static_cast<float>(info.width());
  const float height = static_cast<float>(info.height());
  if (fx <= 0.f || fy <= 0.f || width <= 0.f || height <= 0.f) {
    return plane;
  }

  plane.top_left = FrustumCorner(-cx * far_d / fx, -cy * far_d / fy, far_d);
  plane.top_right =
      FrustumCorner((width - cx) * far_d / fx, -cy * far_d / fy, far_d);
  plane.bottom_right = FrustumCorner((width - cx) * far_d / fx,
                                     (height - cy) * far_d / fy, far_d);
  plane.bottom_left =
      FrustumCorner(-cx * far_d / fx, (height - cy) * far_d / fy, far_d);
  plane.valid = true;
  return plane;
}

QMatrix4x4 opticalToCameraLinkMatrix() {
  return QMatrix4x4(0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

QMatrix4x4 opticalToWorld(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& camera_to_fixed) {
  Q_UNUSED(info);
  return camera_to_fixed * opticalToCameraLinkMatrix();
}

}  // namespace display
}  // namespace autoviz
