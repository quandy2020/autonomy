/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/depth_cloud_utils.hpp"

#include <cmath>
#include <cstring>

namespace autoviz {
namespace display {
namespace {

float ReadDepthMeters(const automsgs::msgs::sensor_msgs::Image& depth, int u,
                      int v) {
  const int width = static_cast<int>(depth.width());
  const int height = static_cast<int>(depth.height());
  if (u < 0 || v < 0 || u >= width || v >= height) {
    return 0.f;
  }
  const size_t index = static_cast<size_t>(v) * static_cast<size_t>(width) +
                       static_cast<size_t>(u);
  const std::string& encoding = depth.encoding();
  const std::string& data = depth.data();

  if (encoding == "32FC1") {
    if (data.size() < (index + 1) * sizeof(float)) {
      return 0.f;
    }
    float value = 0.f;
    std::memcpy(&value, data.data() + index * sizeof(float), sizeof(float));
    return value;
  }
  if (encoding == "16UC1") {
    if (data.size() < (index + 1) * sizeof(uint16_t)) {
      return 0.f;
    }
    uint16_t raw = 0;
    std::memcpy(&raw, data.data() + index * sizeof(uint16_t), sizeof(uint16_t));
    return static_cast<float>(raw) * 0.001f;
  }
  return 0.f;
}

QColor SampleColor(const QImage* image, int u, int v, const QColor& fallback) {
  if (image == nullptr || image->isNull()) {
    return fallback;
  }
  if (u < 0 || v < 0 || u >= image->width() || v >= image->height()) {
    return fallback;
  }
  return QColor(image->pixel(u, v));
}

}  // namespace

std::vector<DepthCloudPoint> projectDepthImage(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QImage* color_image, uint32_t decimation, const QColor& flat_color) {
  std::vector<DepthCloudPoint> points;
  if (info.k_size() < 9 || depth.width() == 0 || depth.height() == 0) {
    return points;
  }

  const float fx = static_cast<float>(info.k(0));
  const float fy = static_cast<float>(info.k(4));
  const float cx = static_cast<float>(info.k(2));
  const float cy = static_cast<float>(info.k(5));
  if (fx <= 0.f || fy <= 0.f) {
    return points;
  }

  const uint32_t step = std::max(1u, decimation);
  const int width = static_cast<int>(depth.width());
  const int height = static_cast<int>(depth.height());
  points.reserve(static_cast<size_t>((width / static_cast<int>(step)) *
                                       (height / static_cast<int>(step))));

  for (int v = 0; v < height; v += static_cast<int>(step)) {
    for (int u = 0; u < width; u += static_cast<int>(step)) {
      const float d = ReadDepthMeters(depth, u, v);
      if (!std::isfinite(d) || d <= 0.01f) {
        continue;
      }
      const float x = (static_cast<float>(u) - cx) * d / fx;
      const float y = (static_cast<float>(v) - cy) * d / fy;
      points.push_back({QVector3D(x, y, d), SampleColor(color_image, u, v, flat_color)});
    }
  }
  return points;
}

}  // namespace display
}  // namespace autoviz
