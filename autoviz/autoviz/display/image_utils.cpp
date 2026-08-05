/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/image_utils.hpp"

#include <algorithm>
#include <cctype>
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

QImage decodeRgba8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_RGBA8888);
  const std::string& data = image.data();
  const int row_bytes = width * 4;
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(row_bytes);
    if (offset + static_cast<size_t>(row_bytes) > data.size()) {
      break;
    }
    std::memcpy(qimage.scanLine(y), data.data() + offset,
                static_cast<size_t>(row_bytes));
  }
  return qimage.convertToFormat(QImage::Format_RGB888);
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

QImage decodeMono16(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_Grayscale8);
  const std::string& data = image.data();
  for (int y = 0; y < height; ++y) {
    auto* dst = qimage.scanLine(y);
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(width) * 2;
    for (int x = 0; x < width; ++x) {
      const size_t index = offset + static_cast<size_t>(x) * 2;
      if (index + 1 >= data.size()) {
        break;
      }
      const uint16_t value =
          static_cast<uint16_t>(static_cast<unsigned char>(data[index])) |
          (static_cast<uint16_t>(static_cast<unsigned char>(data[index + 1])) << 8);
      dst[x] = static_cast<uchar>(value >> 8);
    }
  }
  return qimage;
}

std::string lowerCopy(const std::string& value) {
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

}  // namespace

QImage imageFromProto(const automsgs::msgs::sensor_msgs::Image& image) {
  const std::string encoding = lowerCopy(image.encoding());
  if (encoding == "rgb8" || encoding == "bgr8") {
    QImage decoded = decodeRgb8(image);
    if (encoding == "bgr8" && !decoded.isNull()) {
      decoded = decoded.rgbSwapped();
    }
    return decoded;
  }
  if (encoding == "rgba8" || encoding == "bgra8") {
    QImage decoded = decodeRgba8(image);
    if (encoding == "bgra8" && !decoded.isNull()) {
      decoded = decoded.rgbSwapped();
    }
    return decoded;
  }
  if (encoding == "mono8" || encoding == "8uc1") {
    return decodeMono8(image);
  }
  if (encoding == "mono16" || encoding == "16uc1") {
    return decodeMono16(image);
  }
  return {};
}

QImage compressedImageFromProto(
    const automsgs::msgs::sensor_msgs::CompressedImage& image) {
  if (image.data().empty()) {
    return {};
  }
  QImage decoded;
  if (!decoded.loadFromData(
          reinterpret_cast<const uchar*>(image.data().data()),
          static_cast<int>(image.data().size()))) {
    return {};
  }
  return decoded.convertToFormat(QImage::Format_RGB888);
}

bool isImageMessageType(const std::string& message_type) {
  return message_type == "automsgs.msgs.sensor_msgs.Image" ||
         message_type == "sensor_msgs/Image" ||
         message_type == "automsgs.msgs.sensor_msgs.CompressedImage" ||
         message_type == "sensor_msgs/CompressedImage" ||
         message_type == "foxglove.CompressedVideo" ||
         message_type.find("CompressedVideo") != std::string::npos;
}

}  // namespace display
}  // namespace autoviz
