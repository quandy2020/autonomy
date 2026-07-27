/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/point_cloud_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace autoviz {
namespace display {
namespace {

uint32_t fieldOffset(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    const char* name) {
  for (const auto& field : cloud.fields()) {
    if (field.name() == name) {
      return field.offset();
    }
  }
  return UINT32_MAX;
}

float readFloat32(const uint8_t* base, uint32_t offset) {
  float value = 0.f;
  std::memcpy(&value, base + offset, sizeof(float));
  return value;
}

uint32_t readUint32(const uint8_t* base, uint32_t offset) {
  uint32_t value = 0;
  std::memcpy(&value, base + offset, sizeof(uint32_t));
  return value;
}

}  // namespace

QColor RainbowColor(float t) {
  const float clamped = std::max(0.f, std::min(1.f, t));
  const int r = static_cast<int>(
      255.f * std::min(1.f, std::max(0.f, 1.5f - std::abs(4.f * clamped - 3.f))));
  const int g = static_cast<int>(
      255.f * std::min(1.f, std::max(0.f, 1.5f - std::abs(4.f * clamped - 2.f))));
  const int b = static_cast<int>(
      255.f * std::min(1.f, std::max(0.f, 1.5f - std::abs(4.f * clamped - 1.f))));
  return QColor(r, g, b);
}

const std::array<QColor, 256>& intensityRainbowTable() {
  static const std::array<QColor, 256> kTable = [] {
    std::array<QColor, 256> table{};
    for (int i = 0; i < 256; ++i) {
      table[static_cast<std::size_t>(i)] =
          RainbowColor(static_cast<float>(i) / 255.f);
    }
    return table;
  }();
  return kTable;
}

uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i) {
  if (!std::isfinite(intensity)) {
    return 128;
  }
  const float span = std::max(max_i - min_i, 1e-6f);
  const float t = std::max(0.f, std::min(1.f, (intensity - min_i) / span));
  return static_cast<uint8_t>(std::lround(t * 255.f));
}

QColor colorFromIntensityIndex(uint8_t index) {
  return intensityRainbowTable()[index];
}

QColor colorFromIntensity(float intensity, float min_i, float max_i) {
  return colorFromIntensityIndex(intensityToPaletteIndex(intensity, min_i, max_i));
}

QColor colorFromRgbPacked(uint32_t rgb_packed) {
  const uint8_t r = static_cast<uint8_t>((rgb_packed >> 16) & 0xFF);
  const uint8_t g = static_cast<uint8_t>((rgb_packed >> 8) & 0xFF);
  const uint8_t b = static_cast<uint8_t>(rgb_packed & 0xFF);
  return QColor(r, g, b);
}

ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation) {
  ParsedPointCloud parsed;
  if (cloud.width() == 0 || cloud.point_step() == 0 || cloud.data().empty()) {
    return parsed;
  }

  const uint32_t x_off = fieldOffset(cloud, "x");
  const uint32_t y_off = fieldOffset(cloud, "y");
  const uint32_t z_off = fieldOffset(cloud, "z");
  const uint32_t i_off = fieldOffset(cloud, "intensity");
  const uint32_t rgb_off = fieldOffset(cloud, "rgb");
  const uint32_t rgba_off = rgb_off == UINT32_MAX ? fieldOffset(cloud, "rgba") : rgb_off;
  if (x_off == UINT32_MAX || y_off == UINT32_MAX || z_off == UINT32_MAX) {
    return parsed;
  }

  const std::string& blob_str = cloud.data();
  const auto* blob = reinterpret_cast<const uint8_t*>(blob_str.data());
  const size_t blob_size = blob_str.size();

  const uint32_t count = cloud.width() * std::max(1u, cloud.height());
  const uint32_t step = std::max(1u, decimation);
  parsed.xs.reserve(count / step);
  parsed.ys.reserve(count / step);
  parsed.zs.reserve(count / step);

  for (uint32_t i = 0; i < count; i += step) {
    const size_t base = static_cast<size_t>(i) * cloud.point_step();
    if (base + cloud.point_step() > blob_size) {
      break;
    }
    const uint8_t* ptr = blob + base;
    const float x = readFloat32(ptr, x_off);
    const float y = readFloat32(ptr, y_off);
    const float z = readFloat32(ptr, z_off);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    parsed.xs.push_back(x);
    parsed.ys.push_back(y);
    parsed.zs.push_back(z);
    if (i_off != UINT32_MAX) {
      parsed.intensities.push_back(readFloat32(ptr, i_off));
    }
    if (rgba_off != UINT32_MAX) {
      parsed.rgb.push_back(readUint32(ptr, rgba_off));
    }
  }
  return parsed;
}

}  // namespace display
}  // namespace autoviz
