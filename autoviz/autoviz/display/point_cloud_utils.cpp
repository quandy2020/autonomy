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

/** Find a PointField by name; returns PointFieldInfo with valid=false on miss.
 *  Mirrors rviz_default_plugins::findChannelIndex(). */
PointFieldInfo findField(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    const char* name) {
  for (const auto& field : cloud.fields()) {
    if (field.name() == name) {
      return {field.offset(), static_cast<uint8_t>(field.datatype()), true};
    }
  }
  return {};
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

// Google Turbo colormap (perceptually uniform; good for depth / distance).
// Source: https://gist.github.com/mikhailov-work/0d177465a8151eb6ede1768d51d476c7
QColor TurboColor(float t) {
  const float clamped = std::max(0.f, std::min(1.f, t));
  const float r = 0.1357f + clamped * (4.5974f + clamped * (-42.3277f + clamped * (130.5887f + clamped * (-150.5666f + clamped * 58.1375f))));
  const float g = 0.0914f + clamped * (2.1856f + clamped * (4.8052f  + clamped * (-14.0741f + clamped * (14.3171f  + clamped * -5.3670f))));
  const float b = 0.1070f + clamped * (2.7183f + clamped * (14.4577f + clamped * (-57.8730f + clamped * (91.7920f  + clamped * -54.8728f))));
  return QColor(
      static_cast<int>(std::max(0.f, std::min(255.f, r * 255.f))),
      static_cast<int>(std::max(0.f, std::min(255.f, g * 255.f))),
      static_cast<int>(std::max(0.f, std::min(255.f, b * 255.f))));
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

PointCloudColorMode parsePointCloudColorMode(const std::string& value) {
  if (value == "Intensity") return PointCloudColorMode::kIntensity;
  if (value == "RGB8")      return PointCloudColorMode::kRgb8;
  if (value == "AxisColor X" || value == "Axis X") return PointCloudColorMode::kAxisX;
  if (value == "AxisColor Y" || value == "Axis Y") return PointCloudColorMode::kAxisY;
  if (value == "AxisColor Z" || value == "Axis Z") return PointCloudColorMode::kAxisZ;
  // Legacy "Axis" → Z-axis height coloring (most intuitive default)
  if (value == "Axis") return PointCloudColorMode::kAxisZ;
  return PointCloudColorMode::kFlat;
}

PointCloudColorRamp parsePointCloudColorRamp(const std::string& value) {
  if (value == "Turbo")     return PointCloudColorRamp::kTurbo;
  if (value == "Grayscale") return PointCloudColorRamp::kGrayscale;
  return PointCloudColorRamp::kRainbow;
}

QColor colorFromRamp(float t, PointCloudColorRamp ramp) {
  const float clamped = std::max(0.f, std::min(1.f, t));
  switch (ramp) {
    case PointCloudColorRamp::kTurbo:
      return TurboColor(clamped);
    case PointCloudColorRamp::kGrayscale: {
      const int v = static_cast<int>(clamped * 255.f);
      return QColor(v, v, v);
    }
    case PointCloudColorRamp::kRainbow:
    default:
      return RainbowColor(clamped);
  }
}

QColor colorFromIntensity(float intensity, float min_i, float max_i,
                          PointCloudColorRamp ramp) {
  if (ramp == PointCloudColorRamp::kRainbow) {
    // Fast path: use pre-built palette (256 entries)
    return colorFromIntensityIndex(intensityToPaletteIndex(intensity, min_i, max_i));
  }
  const float span = std::max(max_i - min_i, 1e-6f);
  const float t = std::max(0.f, std::min(1.f, (intensity - min_i) / span));
  return colorFromRamp(t, ramp);
}

ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation) {
  ParsedPointCloud parsed;
  const uint32_t width  = cloud.width();
  const uint32_t height = std::max(1u, cloud.height());
  const uint32_t point_step = cloud.point_step();
  // row_step: bytes per row.  For unorganised clouds (height==1) this equals
  // width*point_step.  For organised clouds it may include row padding.
  // RViz2 xyz_pc_transformer uses pointer arithmetic (+= point_step) which is
  // equivalent to flat addressing when row_step == width*point_step; we handle
  // the general case by computing row/col offsets explicitly — same result for
  // the common flat case, correct for padded organised clouds.
  const uint32_t row_step = cloud.row_step() > 0
      ? cloud.row_step()
      : width * point_step;

  if (width == 0 || point_step == 0 || cloud.data().empty()) {
    return parsed;
  }

  const PointFieldInfo xf = findField(cloud, "x");
  const PointFieldInfo yf = findField(cloud, "y");
  const PointFieldInfo zf = findField(cloud, "z");
  if (!xf.valid || !yf.valid || !zf.valid) {
    return parsed;
  }

  // Optional fields — try "intensity" then "intensities" (mirrors RViz2).
  PointFieldInfo intf = findField(cloud, "intensity");
  if (!intf.valid) {
    intf = findField(cloud, "intensities");
  }
  PointFieldInfo rgbf = findField(cloud, "rgb");
  if (!rgbf.valid) {
    rgbf = findField(cloud, "rgba");
  }

  const std::string& blob_str = cloud.data();
  const auto* blob = reinterpret_cast<const uint8_t*>(blob_str.data());
  const size_t blob_size = blob_str.size();

  const uint32_t count = width * height;
  const uint32_t step  = std::max(1u, decimation);

  parsed.xs.reserve(count / step + 1);
  parsed.ys.reserve(count / step + 1);
  parsed.zs.reserve(count / step + 1);

  const bool has_intensity = intf.valid;
  const bool has_rgb       = rgbf.valid;

  for (uint32_t i = 0; i < count; i += step) {
    // Compute byte offset using row_step — handles organised clouds with
    // row padding correctly (RViz2 equivalent: base = point_step * i when
    // row_step == width * point_step, which is the common case).
    const uint32_t row = i / width;
    const uint32_t col = i % width;
    const size_t   base = static_cast<size_t>(row) * row_step +
                          static_cast<size_t>(col) * point_step;
    if (base + point_step > blob_size) {
      break;
    }
    const uint8_t* ptr = blob + base;

    const float x = valueFromPointData<float>(ptr, xf);
    const float y = valueFromPointData<float>(ptr, yf);
    const float z = valueFromPointData<float>(ptr, zf);

    // Drop NaN/Inf — both RViz2 and sensor_msgs convention treat these as
    // invalid points (lidar returns at max range, occluded pixels, etc.).
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    parsed.xs.push_back(x);
    parsed.ys.push_back(y);
    parsed.zs.push_back(z);

    if (has_intensity) {
      parsed.intensities.push_back(valueFromPointData<float>(ptr, intf));
    }
    if (has_rgb) {
      uint32_t packed = 0;
      std::memcpy(&packed, ptr + rgbf.offset, 4);
      parsed.rgb.push_back(packed);
    }
  }
  return parsed;
}

}  // namespace display
}  // namespace autoviz
