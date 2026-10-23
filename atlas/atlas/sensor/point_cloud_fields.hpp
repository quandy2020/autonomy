/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file point_cloud_fields.hpp
 * @brief PointCloud2 field offsets and decode helpers for Argus point types.
 */

#ifndef ATLAS_SENSOR_POINT_CLOUD_FIELDS_HPP_
#define ATLAS_SENSOR_POINT_CLOUD_FIELDS_HPP_

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "sensor/point_cloud_data.hpp"

namespace atlas {
namespace sensor {

using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;
using PointField = ::automsgs::msgs::sensor_msgs::PointField;

/**
 * @brief Byte offsets into a PointCloud2 point step (-1 if field absent).
 * @var x           Offset of FLOAT32 @c x.
 * @var y           Offset of FLOAT32 @c y.
 * @var z           Offset of FLOAT32 @c z.
 * @var intensity   Offset of a supported scalar numeric intensity field.
 * @var time        Offset of FLOAT32 field @c time (relative s or ms).
 * @var time_uint32 Offset of UINT32 field @c t (often nanoseconds).
 * @var timestamp   Offset of FLOAT64 field @c timestamp.
 * @var r           Offset of red channel field.
 * @var g           Offset of green channel field.
 * @var b           Offset of blue channel field.
 * @var rgb         Offset of packed @c rgb field.
 */
struct FieldOffsets {
  int x = -1;
  int y = -1;
  int z = -1;
  int intensity = -1;
  int intensity_datatype = 0;
  int time = -1;
  int time_uint32 = -1;
  int timestamp = -1;
  int r = -1;
  int g = -1;
  int b = -1;
  int rgb = -1;
};

/**
 * @brief Scan @p msg fields and fill a @ref FieldOffsets map.
 * @param[in] msg Source PointCloud2 metadata and layout.
 * @return Offsets for recognized names/datatypes; missing fields stay -1.
 */
inline FieldOffsets ParseFields(const PointCloud2& msg) {
  FieldOffsets offsets;
  for (const auto& f : msg.fields()) {
    if (f.count() != 1 || f.offset() > static_cast<std::uint32_t>(
                                           std::numeric_limits<int>::max())) {
      continue;
    }
    const int off = static_cast<int>(f.offset());
    if (f.name() == "x" && f.datatype() == PointField::FLOAT32) {
      offsets.x = off;
    } else if (f.name() == "y" && f.datatype() == PointField::FLOAT32) {
      offsets.y = off;
    } else if (f.name() == "z" && f.datatype() == PointField::FLOAT32) {
      offsets.z = off;
    } else if (f.name() == "intensity") {
      offsets.intensity = off;
      offsets.intensity_datatype = static_cast<int>(f.datatype());
    } else if (f.name() == "time" && f.datatype() == PointField::FLOAT32) {
      offsets.time = off;
    } else if (f.name() == "t" && f.datatype() == PointField::UINT32) {
      offsets.time_uint32 = off;
    } else if (f.name() == "timestamp" && f.datatype() == PointField::FLOAT64) {
      offsets.timestamp = off;
    } else if (f.name() == "r" && f.datatype() == PointField::UINT8) {
      offsets.r = off;
    } else if (f.name() == "g" && f.datatype() == PointField::UINT8) {
      offsets.g = off;
    } else if (f.name() == "b" && f.datatype() == PointField::UINT8) {
      offsets.b = off;
    } else if (f.name() == "rgb" && (f.datatype() == PointField::FLOAT32 ||
                                     f.datatype() == PointField::UINT32)) {
      offsets.rgb = off;
    }
  }
  return offsets;
}

/** @brief Check point layout and recognized field bounds before byte reads. */
inline bool IsPointCloud2LayoutValid(const PointCloud2& msg,
                                     const FieldOffsets& offsets) {
  const std::size_t step = static_cast<std::size_t>(msg.point_step());
  const std::size_t width = static_cast<std::size_t>(msg.width());
  const std::size_t height = static_cast<std::size_t>(msg.height());
  const std::size_t row_step = static_cast<std::size_t>(msg.row_step());
  if (step == 0 || width == 0 || height == 0 || row_step == 0 ||
      msg.data().empty() ||
      width > std::numeric_limits<std::size_t>::max() / step ||
      height > std::numeric_limits<std::size_t>::max() / row_step ||
      row_step < width * step || row_step * height > msg.data().size() ||
      msg.is_bigendian()) {
    return false;
  }
  const auto field_fits = [step](int offset, std::size_t size) {
    return offset >= 0 && static_cast<std::size_t>(offset) <= step &&
           size <= step - static_cast<std::size_t>(offset);
  };
  std::size_t intensity_size = 0;
  switch (offsets.intensity_datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      intensity_size = sizeof(std::uint8_t);
      break;
    case PointField::INT16:
    case PointField::UINT16:
      intensity_size = sizeof(std::uint16_t);
      break;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      intensity_size = sizeof(std::uint32_t);
      break;
    case PointField::FLOAT64:
      intensity_size = sizeof(double);
      break;
    default:
      break;
  }
  return field_fits(offsets.x, sizeof(float)) &&
         field_fits(offsets.y, sizeof(float)) &&
         field_fits(offsets.z, sizeof(float)) &&
         (offsets.intensity < 0 ||
          (intensity_size > 0 &&
           field_fits(offsets.intensity, intensity_size))) &&
         (offsets.time < 0 || field_fits(offsets.time, sizeof(float))) &&
         (offsets.time_uint32 < 0 ||
          field_fits(offsets.time_uint32, sizeof(std::uint32_t))) &&
         (offsets.timestamp < 0 ||
          field_fits(offsets.timestamp, sizeof(double))) &&
         (offsets.rgb < 0 || field_fits(offsets.rgb, sizeof(float))) &&
         ((offsets.r < 0 && offsets.g < 0 && offsets.b < 0) ||
          (field_fits(offsets.r, sizeof(std::uint8_t)) &&
           field_fits(offsets.g, sizeof(std::uint8_t)) &&
           field_fits(offsets.b, sizeof(std::uint8_t))));
}

/**
 * @brief Read a trivially copyable value from a point record.
 * @tparam T Stored field type.
 * @param[in] base   Start of the point's byte record.
 * @param[in] offset Byte offset, or negative to skip the read.
 * @return Decoded value, or default-constructed @p T when @p offset is
 *         negative.
 */
template <typename T>
T ReadAt(const char* base, int offset) {
  T value{};
  if (offset >= 0) {
    std::memcpy(&value, base + offset, sizeof(T));
  }
  return value;
}

/** @brief Decode a scalar PointField intensity to float. */
inline float ReadIntensity(const char* base, const FieldOffsets& offsets) {
  switch (offsets.intensity_datatype) {
    case PointField::INT8:
      return static_cast<float>(ReadAt<std::int8_t>(base, offsets.intensity));
    case PointField::UINT8:
      return static_cast<float>(ReadAt<std::uint8_t>(base, offsets.intensity));
    case PointField::INT16:
      return static_cast<float>(ReadAt<std::int16_t>(base, offsets.intensity));
    case PointField::UINT16:
      return static_cast<float>(ReadAt<std::uint16_t>(base, offsets.intensity));
    case PointField::INT32:
      return static_cast<float>(ReadAt<std::int32_t>(base, offsets.intensity));
    case PointField::UINT32:
      return static_cast<float>(ReadAt<std::uint32_t>(base, offsets.intensity));
    case PointField::FLOAT32:
      return ReadAt<float>(base, offsets.intensity);
    case PointField::FLOAT64:
      return static_cast<float>(ReadAt<double>(base, offsets.intensity));
    default:
      return 0.f;
  }
}

/**
 * @brief Decode xyz (required) and optional fields via @p filler into @p cloud.
 * @tparam PointT Argus point type (must expose @c x, @c y, @c z).
 * @tparam Filler Callable
 *         @c void(PointT&, const char* base, const FieldOffsets& offsets).
 * @param[in]  msg    Serialized PointCloud2 payload.
 * @param[out] cloud  Cleared and filled on success; left empty if the layout
 *                    is invalid, unsupported, or empty. Must be non-null.
 * @param[in]  filler Populates extra members (intensity, time, color, …).
 *                    Big-endian PointCloud2 payloads are unsupported.
 */
template <typename PointT, typename Filler>
void DecodeWithFiller(const PointCloud2& msg, LidarCloud<PointT>* cloud,
                      Filler filler) {
  if (cloud == nullptr) {
    return;
  }
  cloud->clear();
  const FieldOffsets offsets = ParseFields(msg);
  if (!IsPointCloud2LayoutValid(msg, offsets)) {
    return;
  }
  const std::size_t step = static_cast<std::size_t>(msg.point_step());
  const std::size_t width = static_cast<std::size_t>(msg.width());
  const std::size_t height = static_cast<std::size_t>(msg.height());
  const std::size_t row_step = static_cast<std::size_t>(msg.row_step());
  const std::string& data = msg.data();
  if (width > cloud->points.max_size() / height) {
    return;
  }
  const std::size_t point_count = width * height;
  if (point_count > std::numeric_limits<std::uint32_t>::max()) {
    return;
  }
  cloud->reserve(point_count);
  for (std::size_t row = 0; row < height; ++row) {
    const std::size_t row_offset = row * row_step;
    for (std::size_t column = 0; column < width; ++column) {
      const char* base = data.data() + row_offset + column * step;
      PointT pt{};
      pt.x = ReadAt<float>(base, offsets.x);
      pt.y = ReadAt<float>(base, offsets.y);
      pt.z = ReadAt<float>(base, offsets.z);
      filler(pt, base, offsets);
      cloud->push_back(pt);
    }
  }
}

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_POINT_CLOUD_FIELDS_HPP_
