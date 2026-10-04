/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file point_cloud_utils.hpp
 * @brief PointCloud2 field parsing and RViz2-compatible color transformers.
 *
 * Provides:
 * - @ref ParsedPointCloud — decoded XYZ + optional intensity / RGB channels
 * - @ref parsePointCloud2 — wire → arrays with optional decimation
 * - @ref PointCloudColorMode and coloring helpers matching
 *   @c rviz_default_plugins point cloud transformers
 *
 * Shared by @ref PointCloud2Display, @ref LaserScanDisplay, and related
 * scalar/point views.
 *
 * @see PointCloud2Display
 * @see getRainbowColor()
 * @see colorFromScalar()
 */

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

/**
 * @struct ParsedPointCloud
 * @brief Parallel arrays of decoded PointCloud2 channels.
 *
 * Length of @c xs / @c ys / @c zs is the point count after decimation.
 * Optional channels may be empty when the cloud lacks those fields.
 */
struct ParsedPointCloud {
  std::vector<float> xs; /**< X positions. */
  std::vector<float> ys; /**< Y positions. */
  std::vector<float> zs; /**< Z positions. */
  /** Scalar channel selected for Intensity transformer. */
  std::vector<float> intensities;
  /** Packed RGB/RGBA (RGB8). Empty unless cloud has rgb/rgba. */
  std::vector<uint32_t> rgb;
  bool rgb_is_rgba = false; /**< When @c true, @c rgb packs RGBA. */
  /** Separate float r/g/b channels (RGBF32). Empty unless all three exist. */
  std::vector<float> r;
  std::vector<float> g;
  std::vector<float> b;
};

/**
 * @brief PointField datatype constants (@c sensor_msgs/PointField.msg).
 *
 * Used by @ref valueFromPointData to interpret field offsets.
 */
namespace PointFieldType {
constexpr uint8_t kINT8 = 1;     /**< Signed 8-bit. */
constexpr uint8_t kUINT8 = 2;    /**< Unsigned 8-bit. */
constexpr uint8_t kINT16 = 3;    /**< Signed 16-bit. */
constexpr uint8_t kUINT16 = 4;   /**< Unsigned 16-bit. */
constexpr uint8_t kINT32 = 5;    /**< Signed 32-bit. */
constexpr uint8_t kUINT32 = 6;   /**< Unsigned 32-bit. */
constexpr uint8_t kFLOAT32 = 7;  /**< IEEE float32. */
constexpr uint8_t kFLOAT64 = 8;  /**< IEEE float64. */
}  // namespace PointFieldType

/**
 * @struct PointFieldInfo
 * @brief Resolved offset + datatype for one PointCloud2 field.
 */
struct PointFieldInfo {
  uint32_t offset = 0;                           /**< Byte offset within a point. */
  uint8_t datatype = PointFieldType::kFLOAT32;   /**< @ref PointFieldType value. */
  bool valid = false;                            /**< Whether the field was found. */
};

/**
 * @brief Reads a typed field from a point's raw bytes (RViz @c valueFromCloud).
 *
 * @tparam T Conversion target type (typically @c float).
 * @param point_ptr Pointer to the start of one point record.
 * @param field Resolved field offset / datatype.
 * @return Value converted to @tparam T, or default-constructed on unknown type.
 *
 * @note Mirrors @c rviz_default_plugins::valueFromCloud<T>().
 */
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

/**
 * @enum PointCloudColorMode
 * @brief RViz2 Color Transformer plugin names
 *        (@c point_cloud_transformer_factory.cpp).
 */
enum class PointCloudColorMode {
  kFlatColor,   /**< FlatColor — single property color. */
  kIntensity,   /**< Intensity — scalar channel → rainbow / ramp. */
  kRgb8,        /**< RGB8 — packed rgb/rgba field. */
  kRgbF32,      /**< RGBF32 — separate float r/g/b. */
  kAxisColor,   /**< AxisColor — color by X/Y/Z (+ Axis property). */
};

/**
 * @brief Decodes a PointCloud2 into parallel float / RGB arrays.
 *
 * @param cloud Source PointCloud2 message.
 * @param decimation Keep every N-th point (@c 1 = all).
 * @param intensity_channel Field name for Intensity transformer; falls back
 *        to @c "intensities" when @p intensity_channel is @c "intensity".
 * @return Parsed arrays (may be empty on failure / empty cloud).
 */
ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation = 1,
    const std::string& intensity_channel = "intensity");

/**
 * @brief Parses a Color Transformer property string into @ref PointCloudColorMode.
 *
 * @param value UI string (e.g. @c "Intensity", @c "RGB8").
 * @return Matching mode; unrecognized values map to a safe default.
 */
PointCloudColorMode parsePointCloudColorMode(const std::string& value);

/**
 * @brief RViz2 @c getRainbowColor (@c point_cloud_helpers.hpp).
 *
 * @param value Normalized scalar in [0, 1].
 * @return Rainbow QColor.
 */
QColor getRainbowColor(float value);

/**
 * @brief Intensity / AxisColor coloring matching RViz2 IntensityPCTransformer.
 *
 * When @p use_rainbow: rainbow (optionally inverted). Else interpolate
 * @p min_color → @p max_color.
 *
 * @param value Scalar to colorize.
 * @param min_v Range minimum.
 * @param max_v Range maximum.
 * @param use_rainbow Prefer rainbow over min/max lerp.
 * @param invert_rainbow Reverse rainbow direction.
 * @param min_color Color at @p min_v when not using rainbow.
 * @param max_color Color at @p max_v when not using rainbow.
 * @return Mapped QColor.
 */
QColor colorFromScalar(float value, float min_v, float max_v, bool use_rainbow,
                       bool invert_rainbow, const QColor& min_color,
                       const QColor& max_color);

/**
 * @brief Unpacks a 24/32-bit RGB(A) field into QColor.
 *
 * @param rgb_packed Packed channel value.
 * @param has_alpha When @c true, interpret as RGBA.
 * @return QColor with components in 0–255.
 */
QColor colorFromRgbPacked(uint32_t rgb_packed, bool has_alpha = false);

/**
 * @brief 256-entry RViz rainbow table (shared with Ogre indexed palette).
 * @return Const reference to the static palette.
 */
const std::array<QColor, 256>& intensityRainbowTable();

/**
 * @brief Maps intensity into a 0–255 palette index.
 *
 * @param intensity Raw intensity.
 * @param min_i Range minimum.
 * @param max_i Range maximum.
 * @return Palette index.
 */
uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i);

/**
 * @brief Looks up @ref intensityRainbowTable by index.
 *
 * @param index Palette index 0–255.
 * @return Table color.
 */
QColor colorFromIntensityIndex(uint8_t index);

/**
 * @brief Legacy intensity→rainbow helper for LaserScan / scalar displays.
 *
 * @param intensity Raw intensity.
 * @param min_i Range minimum.
 * @param max_i Range maximum.
 * @return Rainbow QColor.
 * @see colorFromScalar()
 */
QColor colorFromIntensity(float intensity, float min_i, float max_i);

/**
 * @brief Legacy ramp helper; @p unused retained for call-site compatibility.
 *
 * @param t Normalized parameter in [0, 1].
 * @param unused Ignored (historical API).
 * @return Ramp QColor.
 */
QColor colorFromRamp(float t, int /*unused*/ = 0);

}  // namespace display
}  // namespace autoviz
