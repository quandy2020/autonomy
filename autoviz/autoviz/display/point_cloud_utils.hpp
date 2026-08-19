/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <QColor>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autoviz {
namespace display {

struct ParsedPointCloud {
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  std::vector<float> intensities;
  std::vector<uint32_t> rgb;
};

// PointField datatype constants (sensor_msgs/PointField.msg).
// Matches RViz2 sensor_msgs::msg::PointField values exactly.
namespace PointFieldType {
constexpr uint8_t kINT8    = 1;
constexpr uint8_t kUINT8   = 2;
constexpr uint8_t kINT16   = 3;
constexpr uint8_t kUINT16  = 4;
constexpr uint8_t kINT32   = 5;
constexpr uint8_t kUINT32  = 6;
constexpr uint8_t kFLOAT32 = 7;
constexpr uint8_t kFLOAT64 = 8;
}  // namespace PointFieldType

/** Per-field metadata extracted from PointCloud2 header. */
struct PointFieldInfo {
  uint32_t offset  = 0;
  uint8_t  datatype = PointFieldType::kFLOAT32;
  bool     valid   = false;
};

/** Read a scalar value from a raw point byte pointer, matching all 8 RViz2
 *  PointField datatypes.  Mirrors rviz_default_plugins valueFromCloud<T>(). */
template <typename T>
inline T valueFromPointData(const uint8_t* point_ptr, const PointFieldInfo& field) {
  const uint8_t* data = point_ptr + field.offset;
  switch (field.datatype) {
    case PointFieldType::kINT8:    { int8_t   v; std::memcpy(&v, data, 1); return static_cast<T>(v); }
    case PointFieldType::kUINT8:   { uint8_t  v; std::memcpy(&v, data, 1); return static_cast<T>(v); }
    case PointFieldType::kINT16:   { int16_t  v; std::memcpy(&v, data, 2); return static_cast<T>(v); }
    case PointFieldType::kUINT16:  { uint16_t v; std::memcpy(&v, data, 2); return static_cast<T>(v); }
    case PointFieldType::kINT32:   { int32_t  v; std::memcpy(&v, data, 4); return static_cast<T>(v); }
    case PointFieldType::kUINT32:  { uint32_t v; std::memcpy(&v, data, 4); return static_cast<T>(v); }
    case PointFieldType::kFLOAT32: { float    v; std::memcpy(&v, data, 4); return static_cast<T>(v); }
    case PointFieldType::kFLOAT64: { double   v; std::memcpy(&v, data, 8); return static_cast<T>(v); }
    default: return T{};
  }
}

enum class PointCloudColorMode {
  kFlat,       ///< Uniform color (RViz2 "Flat Color", Foxglove "Flat")
  kIntensity,  ///< Scalar field → color ramp (RViz2 "Intensity")
  kRgb8,       ///< Packed RGB/RGBA field (RViz2 "RGB8")
  kAxisX,      ///< World-X range → color ramp (RViz2 "AxisColor X")
  kAxisY,      ///< World-Y range → color ramp
  kAxisZ,      ///< World-Z range → color ramp (most common: height coloring)
};

/** Color ramp used for Intensity / Axis color transformers. */
enum class PointCloudColorRamp {
  kRainbow,    ///< Classic HSV rainbow (RViz2 default)
  kTurbo,      ///< Google Turbo: perceptually uniform, good for depth
  kGrayscale,  ///< White → Black
};

/** Decode commsgs PointCloud2 into xyz arrays with optional intensity/rgb. */
ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation = 1);

/** Parse color-transformer UI string (RViz2 / Foxglove naming). */
PointCloudColorMode parsePointCloudColorMode(const std::string& value);

/** Parse color-ramp UI string. */
PointCloudColorRamp parsePointCloudColorRamp(const std::string& value);

/** Map a normalized scalar t∈[0,1] through the chosen color ramp. */
QColor colorFromRamp(float t, PointCloudColorRamp ramp);

QColor colorFromIntensity(float intensity, float min_i, float max_i,
                          PointCloudColorRamp ramp = PointCloudColorRamp::kRainbow);
/** 256-entry rainbow table shared with Ogre indexed_8bit palette. */
const std::array<QColor, 256>& intensityRainbowTable();
uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i);
QColor colorFromIntensityIndex(uint8_t index);
QColor colorFromRgbPacked(uint32_t rgb_packed);

}  // namespace display
}  // namespace autoviz
