/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_point_cloud_projection.hpp"

#include <algorithm>
#include <cmath>

#include <QQuaternion>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/point_cloud_utils.hpp"

namespace autoviz {
namespace image {
namespace {

QMatrix4x4 TransformToMatrix(
    const automsgs::msgs::geometry_msgs::Transform& transform) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(static_cast<float>(transform.translation().x()),
                   static_cast<float>(transform.translation().y()),
                   static_cast<float>(transform.translation().z()));
  matrix.rotate(
      QQuaternion(static_cast<float>(transform.rotation().w()),
                  static_cast<float>(transform.rotation().x()),
                  static_cast<float>(transform.rotation().y()),
                  static_cast<float>(transform.rotation().z())));
  return matrix;
}

QColor ColorFromNormalized(float t) {
  return display::getRainbowColor(std::clamp(t, 0.0f, 1.0f));
}

}  // namespace

ImageAnnotationLayer projectPointCloudToLayer(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const std::string& fixed_frame, autoviz::transform::Buffer* tf_buffer,
    int max_points) {
  ImageAnnotationLayer layer;
  if (!intrinsics.valid || tf_buffer == nullptr || max_points <= 0) {
    return layer;
  }

  const uint32_t width = cloud.width() > 0 ? cloud.width() : 1;
  const uint32_t height = cloud.height() > 0 ? cloud.height() : 1;
  const uint64_t total = static_cast<uint64_t>(width) * height;
  uint32_t decimation = 1;
  if (total > static_cast<uint64_t>(max_points)) {
    decimation = static_cast<uint32_t>(
        (total + static_cast<uint64_t>(max_points) - 1) /
        static_cast<uint64_t>(max_points));
  }

  const display::ParsedPointCloud parsed =
      display::parsePointCloud2(cloud, std::max<uint32_t>(1, decimation));
  if (parsed.xs.empty()) {
    return layer;
  }

  QMatrix4x4 cloud_to_fixed;
  cloud_to_fixed.setToIdentity();
  try {
    const auto zero_time = autoviz::commsgs::ZeroTime();
    const auto transform = tf_buffer->lookupTransform(
        fixed_frame, cloud.header().frame_id(), zero_time);
    cloud_to_fixed = TransformToMatrix(transform.transform());
  } catch (...) {
    cloud_to_fixed.setToIdentity();
  }
  const QMatrix4x4 cloud_to_optical = fixed_to_optical * cloud_to_fixed;

  float min_scalar = 0.0f;
  float max_scalar = 1.0f;
  bool have_intensity = !parsed.intensities.empty() &&
                        parsed.intensities.size() == parsed.xs.size();
  if (have_intensity) {
    min_scalar = parsed.intensities.front();
    max_scalar = parsed.intensities.front();
    for (float v : parsed.intensities) {
      min_scalar = std::min(min_scalar, v);
      max_scalar = std::max(max_scalar, v);
    }
    if (max_scalar - min_scalar < 1e-6f) {
      max_scalar = min_scalar + 1.0f;
    }
  }

  layer.points.reserve(static_cast<int>(parsed.xs.size()));
  for (std::size_t i = 0; i < parsed.xs.size(); ++i) {
    const QVector3D local(parsed.xs[i], parsed.ys[i], parsed.zs[i]);
    const QVector3D optical = cloud_to_optical.map(local);
    if (optical.z() <= 1e-4f) {
      continue;
    }
    const auto pixel = projectOpticalPointToPixel(intrinsics, optical);
    if (!pixel.has_value()) {
      continue;
    }
    ImageAnnotationPoint point;
    point.position = *pixel;
    point.size = 2.0f;
    if (have_intensity) {
      const float t =
          (parsed.intensities[i] - min_scalar) / (max_scalar - min_scalar);
      point.color = ColorFromNormalized(t);
    } else {
      const float t = std::clamp(optical.z() / 20.0f, 0.0f, 1.0f);
      point.color = ColorFromNormalized(t);
    }
    layer.points.push_back(point);
  }
  layer.timestamp_ns =
      static_cast<qint64>(cloud.header().stamp().sec()) * 1000000000LL +
      static_cast<qint64>(cloud.header().stamp().nanosec());
  return layer;
}

}  // namespace image
}  // namespace autoviz
