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
 * @brief PointCloud2 field offsets + decode helpers for Argus sensor types.
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_FIELDS_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_FIELDS_HPP_

#include <cstddef>
#include <cstring>
#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace sensor {

using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;
using PointField = ::automsgs::msgs::sensor_msgs::PointField;

/** @brief Byte offsets into a PointCloud2 point step (-1 if field absent). */
struct FieldOffsets {
    int x = -1;
    int y = -1;
    int z = -1;
    int intensity = -1;
    /** @brief PointCloud2 field `time` (FLOAT32, relative seconds or ms). */
    int time = -1;
    /** @brief PointCloud2 field `t` (UINT32, often nanoseconds). */
    int time_uint32 = -1;
    /** @brief PointCloud2 field `timestamp` (FLOAT64). */
    int timestamp = -1;
    int r = -1;
    int g = -1;
    int b = -1;
    int rgb = -1;
};

/**
 * @brief Scan @p msg fields and fill a @ref FieldOffsets map.
 * @param msg Source PointCloud2 metadata and layout.
 * @return Offsets for recognized names/datatypes; missing fields stay -1.
 */
inline FieldOffsets ParseFields(const PointCloud2& msg) {
    FieldOffsets offsets;
    for (const auto& f : msg.fields()) {
        const int off = static_cast<int>(f.offset());
        if (f.name() == "x" && f.datatype() == PointField::FLOAT32) {
            offsets.x = off;
        } else if (f.name() == "y" && f.datatype() == PointField::FLOAT32) {
            offsets.y = off;
        } else if (f.name() == "z" && f.datatype() == PointField::FLOAT32) {
            offsets.z = off;
        } else if (f.name() == "intensity") {
            offsets.intensity = off;
        } else if (f.name() == "time" && f.datatype() == PointField::FLOAT32) {
            offsets.time = off;
        } else if (f.name() == "t" && f.datatype() == PointField::UINT32) {
            offsets.time_uint32 = off;
        } else if (f.name() == "timestamp" &&
                   f.datatype() == PointField::FLOAT64) {
            offsets.timestamp = off;
        } else if (f.name() == "r") {
            offsets.r = off;
        } else if (f.name() == "g") {
            offsets.g = off;
        } else if (f.name() == "b") {
            offsets.b = off;
        } else if (f.name() == "rgb") {
            offsets.rgb = off;
        }
    }
    return offsets;
}

/**
 * @brief Read a trivially copyable value from a point record.
 * @tparam T Stored field type.
 * @param base Start of the point's byte record.
 * @param offset Byte offset, or negative to return default-constructed @p T.
 * @return Decoded value, or zero/default if @p offset is negative.
 */
template <typename T>
T ReadAt(const char* base, int offset) {
    T value{};
    if (offset >= 0) {
        std::memcpy(&value, base + offset, sizeof(T));
    }
    return value;
}

/**
 * @brief Decode xyz (required) and optional fields via @p filler into @p cloud.
 * @tparam PointT Argus point type (must accept x, y, z).
 * @tparam Filler Callable `void(PointT&, const char* base, const FieldOffsets&
 * offsets)`.
 * @param msg Serialized PointCloud2 payload.
 * @param[out] cloud Cleared and filled on success; unchanged if xyz missing or
 * empty data.
 * @param filler Populates extra members (intensity, time, color, …).
 */
template <typename PointT, typename Filler>
void DecodeWithFiller(const PointCloud2& msg, PointCloud<PointT>* cloud,
                      Filler filler) {
    cloud->clear();
    if (msg.point_step() == 0 || msg.data().empty()) {
        return;
    }
    const FieldOffsets offsets = ParseFields(msg);
    if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0) {
        return;
    }
    const auto step = static_cast<std::size_t>(msg.point_step());
    const std::string& data = msg.data();
    cloud->reserve(data.size() / step);
    for (std::size_t i = 0; i + step <= data.size(); i += step) {
        const char* base = data.data() + i;
        PointT pt{};
        pt.x = ReadAt<float>(base, offsets.x);
        pt.y = ReadAt<float>(base, offsets.y);
        pt.z = ReadAt<float>(base, offsets.z);
        filler(pt, base, offsets);
        cloud->push_back(pt);
    }
}

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_FIELDS_HPP_
