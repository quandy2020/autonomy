/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/image_utils.hpp"

#include <cstring>

namespace autoviz {
namespace display {
namespace {

QImage decodeRgb8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_RGB888);
  const std::string& data = image.data();
  const int row_bytes = width * 3;
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(row_bytes);
    if (offset + static_cast<size_t>(row_bytes) > data.size()) {
      break;
    }
    std::memcpy(qimage.scanLine(y), data.data() + offset,
                static_cast<size_t>(row_bytes));
  }
  return qimage;
}

QImage decodeMono8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_Grayscale8);
  const std::string& data = image.data();
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(width);
    if (offset + static_cast<size_t>(width) > data.size()) {
      break;
    }
    std::memcpy(qimage.scanLine(y), data.data() + offset,
                static_cast<size_t>(width));
  }
  return qimage;
}

}  // namespace

QImage imageFromProto(
    const automsgs::msgs::sensor_msgs::Image& image) {
  const std::string& encoding = image.encoding();
  if (encoding == "rgb8" || encoding == "bgr8") {
    QImage decoded = decodeRgb8(image);
    if (encoding == "bgr8" && !decoded.isNull()) {
      decoded = decoded.rgbSwapped();
    }
    return decoded;
  }
  if (encoding == "mono8" || encoding == "8UC1") {
    return decodeMono8(image);
  }
  return {};
}

}  // namespace display
}  // namespace autoviz
