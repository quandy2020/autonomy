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

PointFieldInfo findIntensityField(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    const std::string& channel) {
  PointFieldInfo field = findField(cloud, channel.c_str());
  if (!field.valid && channel == "intensity") {
    field = findField(cloud, "intensities");
  }
  return field;
}

}  // namespace

// Matches rviz_default_plugins::getRainbowColor (point_cloud_helpers.hpp).
QColor getRainbowColor(float value) {
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  const float h = value * 5.0f + 1.0f;
  int i = static_cast<int>(std::floor(h));
  float f = h - static_cast<float>(i);
  if (!(i & 1)) {
    f = 1.0f - f;
  }
  const float n = 1.0f - f;

  float r = 0.f, g = 0.f, b = 0.f;
  if (i <= 1) {
    r = n;
    g = 0.f;
    b = 1.f;
  } else if (i == 2) {
    r = 0.f;
    g = n;
    b = 1.f;
  } else if (i == 3) {
    r = 0.f;
    g = 1.f;
    b = n;
  } else if (i == 4) {
    r = n;
    g = 1.f;
    b = 0.f;
  } else {
    r = 1.f;
    g = n;
    b = 0.f;
  }
  return QColor(static_cast<int>(r * 255.f + 0.5f),
                static_cast<int>(g * 255.f + 0.5f),
                static_cast<int>(b * 255.f + 0.5f));
}

const std::array<QColor, 256>& intensityRainbowTable() {
  static const std::array<QColor, 256> kTable = [] {
    std::array<QColor, 256> table{};
    for (int i = 0; i < 256; ++i) {
      table[static_cast<std::size_t>(i)] =
          getRainbowColor(static_cast<float>(i) / 255.f);
    }
    return table;
  }();
  return kTable;
}

uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i) {
  if (!std::isfinite(intensity)) {
    return 128;
  }
  float diff = max_i - min_i;
  if (diff == 0.f) {
    diff = 1e20f;
  }
  // RViz Intensity: value = 1 - (val - min) / diff  (high intensity → cool end)
  float value = 1.0f - (intensity - min_i) / diff;
  value = std::max(0.f, std::min(1.f, value));
  return static_cast<uint8_t>(std::lround(value * 255.f));
}

QColor colorFromIntensityIndex(uint8_t index) {
  return intensityRainbowTable()[index];
}

QColor colorFromIntensity(float intensity, float min_i, float max_i) {
  return colorFromIntensityIndex(intensityToPaletteIndex(intensity, min_i, max_i));
}

QColor colorFromRamp(float t, int) {
  return getRainbowColor(t);
}

QColor colorFromScalar(float value, float min_v, float max_v, bool use_rainbow,
                       bool invert_rainbow, const QColor& min_color,
                       const QColor& max_color) {
  float diff = max_v - min_v;
  if (diff == 0.f) {
    // RViz: when min==max, treat as huge so normalized ≈ 0 → uniform color.
    diff = 1e20f;
  }

  if (use_rainbow) {
    float t = 1.0f - (value - min_v) / diff;
    if (invert_rainbow) {
      t = 1.0f - t;
    }
    t = std::max(0.f, std::min(1.f, t));
    return getRainbowColor(t);
  }

  float normalized = (value - min_v) / diff;
  normalized = std::min(1.0f, std::max(0.0f, normalized));
  const float inv = 1.0f - normalized;
  return QColor(
      static_cast<int>(max_color.red() * normalized + min_color.red() * inv + 0.5f),
      static_cast<int>(max_color.green() * normalized + min_color.green() * inv +
                       0.5f),
      static_cast<int>(max_color.blue() * normalized + min_color.blue() * inv +
                       0.5f));
}

QColor colorFromRgbPacked(uint32_t rgb_packed, bool has_alpha) {
  const uint8_t r = static_cast<uint8_t>((rgb_packed >> 16) & 0xFF);
  const uint8_t g = static_cast<uint8_t>((rgb_packed >> 8) & 0xFF);
  const uint8_t b = static_cast<uint8_t>(rgb_packed & 0xFF);
  QColor color(r, g, b);
  if (has_alpha) {
    color.setAlpha(static_cast<int>((rgb_packed >> 24) & 0xFF));
  }
  return color;
}

PointCloudColorMode parsePointCloudColorMode(const std::string& value) {
  if (value == "Intensity") {
    return PointCloudColorMode::kIntensity;
  }
  if (value == "RGB8") {
    return PointCloudColorMode::kRgb8;
  }
  if (value == "RGBF32") {
    return PointCloudColorMode::kRgbF32;
  }
  if (value == "AxisColor" || value == "Axis" || value == "AxisColor Z" ||
      value == "Axis Z" || value == "AxisColor X" || value == "Axis X" ||
      value == "AxisColor Y" || value == "Axis Y") {
    return PointCloudColorMode::kAxisColor;
  }
  // FlatColor / Flat / Flat Color / legacy defaults
  return PointCloudColorMode::kFlatColor;
}

ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud, uint32_t decimation,
    const std::string& intensity_channel) {
  ParsedPointCloud parsed;
  const uint32_t width = cloud.width();
  const uint32_t height = std::max(1u, cloud.height());
  const uint32_t point_step = cloud.point_step();
  const uint32_t row_step =
      cloud.row_step() > 0 ? cloud.row_step() : width * point_step;

  if (width == 0 || point_step == 0 || cloud.data().empty()) {
    return parsed;
  }

  const PointFieldInfo xf = findField(cloud, "x");
  const PointFieldInfo yf = findField(cloud, "y");
  const PointFieldInfo zf = findField(cloud, "z");
  if (!xf.valid || !yf.valid || !zf.valid) {
    return parsed;
  }

  const PointFieldInfo intf = findIntensityField(cloud, intensity_channel);
  PointFieldInfo rgbf = findField(cloud, "rgb");
  bool rgba = false;
  if (!rgbf.valid) {
    rgbf = findField(cloud, "rgba");
    rgba = rgbf.valid;
  }
  const PointFieldInfo rf = findField(cloud, "r");
  const PointFieldInfo gf = findField(cloud, "g");
  const PointFieldInfo bf = findField(cloud, "b");
  const bool has_rgbf32 =
      rf.valid && gf.valid && bf.valid &&
      rf.datatype == PointFieldType::kFLOAT32;

  const std::string& blob_str = cloud.data();
  const auto* blob = reinterpret_cast<const uint8_t*>(blob_str.data());
  const size_t blob_size = blob_str.size();

  const uint32_t count = width * height;
  const uint32_t step = std::max(1u, decimation);

  parsed.xs.reserve(count / step + 1);
  parsed.ys.reserve(count / step + 1);
  parsed.zs.reserve(count / step + 1);

  for (uint32_t i = 0; i < count; i += step) {
    const uint32_t row = i / width;
    const uint32_t col = i % width;
    const size_t base = static_cast<size_t>(row) * row_step +
                        static_cast<size_t>(col) * point_step;
    if (base + point_step > blob_size) {
      break;
    }
    const uint8_t* ptr = blob + base;

    const float x = valueFromPointData<float>(ptr, xf);
    const float y = valueFromPointData<float>(ptr, yf);
    const float z = valueFromPointData<float>(ptr, zf);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    parsed.xs.push_back(x);
    parsed.ys.push_back(y);
    parsed.zs.push_back(z);

    if (intf.valid) {
      parsed.intensities.push_back(valueFromPointData<float>(ptr, intf));
    }
    if (rgbf.valid) {
      uint32_t packed = 0;
      std::memcpy(&packed, ptr + rgbf.offset, 4);
      parsed.rgb.push_back(packed);
    }
    if (has_rgbf32) {
      parsed.r.push_back(valueFromPointData<float>(ptr, rf));
      parsed.g.push_back(valueFromPointData<float>(ptr, gf));
      parsed.b.push_back(valueFromPointData<float>(ptr, bf));
    }
  }

  parsed.rgb_is_rgba = rgba && !parsed.rgb.empty();
  return parsed;
}

}  // namespace display
}  // namespace autoviz
