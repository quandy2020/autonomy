/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <QColor>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autoviz {
namespace display {

struct ParsedPointCloud {
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  /** Scalar channel selected for Intensity transformer. */
  std::vector<float> intensities;
  /** Packed RGB/RGBA (RGB8). Empty unless cloud has rgb/rgba. */
  std::vector<uint32_t> rgb;
  bool rgb_is_rgba = false;
  /** Separate float r/g/b channels (RGBF32). Empty unless all three exist. */
  std::vector<float> r;
  std::vector<float> g;
  std::vector<float> b;
};

// PointField datatype constants (sensor_msgs/PointField.msg).
namespace PointFieldType {
constexpr uint8_t kINT8 = 1;
constexpr uint8_t kUINT8 = 2;
constexpr uint8_t kINT16 = 3;
constexpr uint8_t kUINT16 = 4;
constexpr uint8_t kINT32 = 5;
constexpr uint8_t kUINT32 = 6;
constexpr uint8_t kFLOAT32 = 7;
constexpr uint8_t kFLOAT64 = 8;
}  // namespace PointFieldType

struct PointFieldInfo {
  uint32_t offset = 0;
  uint8_t datatype = PointFieldType::kFLOAT32;
  bool valid = false;
};

/** Mirrors rviz_default_plugins::valueFromCloud<T>(). */
template <typename T>
inline T valueFromPointData(const uint8_t* point_ptr, const PointFieldInfo& field) {
  const uint8_t* data = point_ptr + field.offset;
  switch (field.datatype) {
    case PointFieldType::kINT8: {
      int8_t v;
      std::memcpy(&v, data, 1);
      return static_cast<T>(v);
    }
    case PointFieldType::kUINT8: {
      uint8_t v;
      std::memcpy(&v, data, 1);
      return static_cast<T>(v);
    }
    case PointFieldType::kINT16: {
      int16_t v;
      std::memcpy(&v, data, 2);
      return static_cast<T>(v);
    }
    case PointFieldType::kUINT16: {
      uint16_t v;
      std::memcpy(&v, data, 2);
      return static_cast<T>(v);
    }
    case PointFieldType::kINT32: {
      int32_t v;
      std::memcpy(&v, data, 4);
      return static_cast<T>(v);
    }
    case PointFieldType::kUINT32: {
      uint32_t v;
      std::memcpy(&v, data, 4);
      return static_cast<T>(v);
    }
    case PointFieldType::kFLOAT32: {
      float v;
      std::memcpy(&v, data, 4);
      return static_cast<T>(v);
    }
    case PointFieldType::kFLOAT64: {
      double v;
      std::memcpy(&v, data, 8);
      return static_cast<T>(v);
    }
    default:
      return T{};
  }
}

/** RViz2 Color Transformer plugin names (point_cloud_transformer_factory.cpp). */
enum class PointCloudColorMode {
  kFlatColor,   ///< FlatColor
  kIntensity,   ///< Intensity
  kRgb8,        ///< RGB8
  kRgbF32,      ///< RGBF32
  kAxisColor,   ///< AxisColor (+ Axis X/Y/Z)
};

/** Decode PointCloud2. intensity_channel selects Intensity transformer field
 *  (falls back to "intensities" when channel is "intensity"). */
ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation = 1,
    const std::string& intensity_channel = "intensity");

PointCloudColorMode parsePointCloudColorMode(const std::string& value);

/** RViz2 getRainbowColor (point_cloud_helpers.hpp). value in [0,1]. */
QColor getRainbowColor(float value);

/** Intensity / AxisColor coloring matching RViz2 IntensityPCTransformer.
 *  When use_rainbow: rainbow (optionally inverted). Else interpolate min→max. */
QColor colorFromScalar(float value, float min_v, float max_v, bool use_rainbow,
                       bool invert_rainbow, const QColor& min_color,
                       const QColor& max_color);

QColor colorFromRgbPacked(uint32_t rgb_packed, bool has_alpha = false);

/** 256-entry RViz rainbow table (shared with Ogre indexed palette). */
const std::array<QColor, 256>& intensityRainbowTable();
uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i);
QColor colorFromIntensityIndex(uint8_t index);

/** Legacy helpers kept for LaserScan / scalar displays. */
QColor colorFromIntensity(float intensity, float min_i, float max_i);
QColor colorFromRamp(float t, int /*unused*/ = 0);

}  // namespace display
}  // namespace autoviz
