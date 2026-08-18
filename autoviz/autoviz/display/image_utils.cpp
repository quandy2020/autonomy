/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/image_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <limits>
#include <vector>

#include <QRgb>
#include <QtGlobal>

namespace autoviz {
namespace display {
namespace {

QImage decodeRgb8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  const int step =
      image.step() > 0 ? static_cast<int>(image.step()) : width * 3;
  if (step < width * 3) {
    return {};
  }
  // 32-bit RGB: Qt's raster engine (and QPixmap/X11) drops tiles for
  // packed 24-bit Format_RGB888, which shows up as solid color blocks.
  QImage qimage(width, height, QImage::Format_RGB32);
  qimage.fill(qRgb(0, 0, 0));
  const std::string& data = image.data();
  const int row_bytes = width * 3;
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(step);
    if (offset + static_cast<size_t>(row_bytes) > data.size()) {
      break;
    }
    const auto* src =
        reinterpret_cast<const unsigned char*>(data.data() + offset);
    auto* dst = reinterpret_cast<QRgb*>(qimage.scanLine(y));
    for (int x = 0; x < width; ++x) {
      dst[x] = qRgb(src[0], src[1], src[2]);
      src += 3;
    }
  }
  return qimage;
}

QImage decodeRgba8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  const int step =
      image.step() > 0 ? static_cast<int>(image.step()) : width * 4;
  if (step < width * 4) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_RGB32);
  qimage.fill(qRgb(0, 0, 0));
  const std::string& data = image.data();
  const int row_bytes = width * 4;
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(step);
    if (offset + static_cast<size_t>(row_bytes) > data.size()) {
      break;
    }
    const auto* src =
        reinterpret_cast<const unsigned char*>(data.data() + offset);
    auto* dst = reinterpret_cast<QRgb*>(qimage.scanLine(y));
    for (int x = 0; x < width; ++x) {
      dst[x] = qRgb(src[0], src[1], src[2]);
      src += 4;
    }
  }
  return qimage;
}

QImage decodeMono8(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  const int step =
      image.step() > 0 ? static_cast<int>(image.step()) : width;
  if (step < width) {
    return {};
  }
  QImage qimage(width, height, QImage::Format_Grayscale8);
  const std::string& data = image.data();
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(step);
    if (offset + static_cast<size_t>(width) > data.size()) {
      break;
    }
    std::memcpy(qimage.scanLine(y), data.data() + offset,
                static_cast<size_t>(width));
  }
  return qimage;
}

// Foxglove Image panel Color map = Turbo (Google turbo polynomial).
void TurboRgb(float t, uchar* rgb) {
  t = std::clamp(t, 0.f, 1.f);
  const float t2 = t * t;
  const float t3 = t2 * t;
  const float t4 = t2 * t2;
  const float t5 = t4 * t;
  const float r = 0.13572138f + 4.61539260f * t - 42.66032258f * t2 +
                  132.13108234f * t3 - 152.94239396f * t4 + 59.28637943f * t5;
  const float g = 0.09140261f + 2.19418839f * t + 4.84296658f * t2 -
                  14.18503333f * t3 + 4.27729857f * t4 + 2.82956604f * t5;
  const float b = 0.10667330f + 12.64194608f * t - 60.58204836f * t2 +
                  110.36276771f * t3 - 89.90310912f * t4 + 27.34824973f * t5;
  rgb[0] = static_cast<uchar>(std::clamp(r, 0.f, 1.f) * 255.f);
  rgb[1] = static_cast<uchar>(std::clamp(g, 0.f, 1.f) * 255.f);
  rgb[2] = static_cast<uchar>(std::clamp(b, 0.f, 1.f) * 255.f);
}

QImage colorizeDepthValues(const std::vector<float>& values, int width,
                           int height) {
  float min_value = std::numeric_limits<float>::max();
  float max_value = std::numeric_limits<float>::lowest();
  for (float value : values) {
    if (std::isfinite(value) && value > 0.f) {
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
  }
  QImage qimage(width, height, QImage::Format_RGB32);
  qimage.fill(qRgb(0, 0, 0));
  if (!(max_value > min_value)) {
    return qimage;
  }
  const float span = max_value - min_value;
  for (int y = 0; y < height; ++y) {
    auto* dst = reinterpret_cast<QRgb*>(qimage.scanLine(y));
    for (int x = 0; x < width; ++x) {
      const float value =
          values[static_cast<size_t>(y) * static_cast<size_t>(width) +
                 static_cast<size_t>(x)];
      if (!std::isfinite(value) || value <= 0.f) {
        continue;
      }
      uchar rgb[3];
      TurboRgb((value - min_value) / span, rgb);
      dst[x] = qRgb(rgb[0], rgb[1], rgb[2]);
    }
  }
  return qimage;
}

QImage decodeMono16(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  const int step =
      image.step() > 0 ? static_cast<int>(image.step()) : width * 2;
  if (step < width * 2) {
    return {};
  }
  std::vector<float> values(static_cast<size_t>(width) * static_cast<size_t>(height),
                            0.f);
  const std::string& data = image.data();
  for (int y = 0; y < height; ++y) {
    const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(step);
    for (int x = 0; x < width; ++x) {
      const size_t index = offset + static_cast<size_t>(x) * 2;
      if (index + 1 >= data.size()) {
        break;
      }
      const uint16_t raw =
          static_cast<uint16_t>(static_cast<unsigned char>(data[index])) |
          (static_cast<uint16_t>(static_cast<unsigned char>(data[index + 1])) << 8);
      values[static_cast<size_t>(y) * static_cast<size_t>(width) +
             static_cast<size_t>(x)] = static_cast<float>(raw);
    }
  }
  return colorizeDepthValues(values, width, height);
}

QImage decodeDepth32F(const automsgs::msgs::sensor_msgs::Image& image) {
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  if (width <= 0 || height <= 0 || image.data().empty()) {
    return {};
  }
  const int step =
      image.step() > 0 ? static_cast<int>(image.step()) : width * static_cast<int>(sizeof(float));
  if (step < width * static_cast<int>(sizeof(float))) {
    return {};
  }
  const std::string& data = image.data();
  const size_t expected =
      static_cast<size_t>(height - 1) * static_cast<size_t>(step) +
      static_cast<size_t>(width) * sizeof(float);
  if (data.size() < expected) {
    return {};
  }

  std::vector<float> values(static_cast<size_t>(width) * static_cast<size_t>(height), 0.f);
  for (int y = 0; y < height; ++y) {
    const size_t row_offset = static_cast<size_t>(y) * static_cast<size_t>(step);
    for (int x = 0; x < width; ++x) {
      const size_t index = row_offset + static_cast<size_t>(x) * sizeof(float);
      float value = 0.f;
      std::memcpy(&value, data.data() + index, sizeof(float));
      values[static_cast<size_t>(y) * static_cast<size_t>(width) +
             static_cast<size_t>(x)] = value;
    }
  }
  return colorizeDepthValues(values, width, height);
}

QImage toDisplayRgbImage(const QImage& image) {
  if (image.isNull()) {
    return {};
  }
  if (image.format() == QImage::Format_RGB32) {
    return image;
  }
  return image.convertToFormat(QImage::Format_RGB32);
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
  QImage decoded;
  if (encoding == "rgb8" || encoding == "bgr8") {
    decoded = decodeRgb8(image);
    if (encoding == "bgr8" && !decoded.isNull()) {
      decoded = decoded.rgbSwapped();
    }
  } else if (encoding == "rgba8" || encoding == "bgra8") {
    decoded = decodeRgba8(image);
    if (encoding == "bgra8" && !decoded.isNull()) {
      decoded = decoded.rgbSwapped();
    }
  } else if (encoding == "mono8" || encoding == "8uc1") {
    decoded = decodeMono8(image);
  } else if (encoding == "mono16" || encoding == "16uc1") {
    decoded = decodeMono16(image);
  } else if (encoding == "32fc1") {
    decoded = decodeDepth32F(image);
  }
  return toDisplayRgbImage(decoded);
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
  return decoded.convertToFormat(QImage::Format_RGB32);
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
