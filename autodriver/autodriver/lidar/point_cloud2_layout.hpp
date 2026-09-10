/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Shared sensor_msgs/PointCloud2 layout: x,y,z,intensity,timestamp.
 *
 * Velodyne / Hesai / Livox convert paths all emit point_step=24 with the same
 * field offsets. Vendor geometry stays in each convert.cpp; only the message
 * envelope is shared here.
 */

#ifndef AUTODRIVER_LIDAR_POINT_CLOUD2_LAYOUT_HPP_
#define AUTODRIVER_LIDAR_POINT_CLOUD2_LAYOUT_HPP_

#include <cstdint>
#include <cstring>
#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autodriver {
namespace lidar {

/** Byte size of one XYZIT point (float32×4 + float64). */
inline constexpr std::uint32_t kXyzitPointStep = 24;

/**
 * @brief Append a PointField descriptor to @p cloud.
 * @param cloud Target PointCloud2; must be non-null.
 * @param name Field name (e.g. "x").
 * @param offset Byte offset within each point.
 * @param datatype PointField data type enum.
 * @param count Number of elements (default 1).
 */
inline void AddPointField(
    automsgs::msgs::sensor_msgs::PointCloud2* cloud, const std::string& name,
    std::uint32_t offset,
    automsgs::msgs::sensor_msgs::PointField::DataType datatype,
    std::uint32_t count = 1) {
    auto* field = cloud->add_fields();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(datatype);
    field->set_count(count);
}

/**
 * @brief Initialize an empty XYZIT PointCloud2 (fields + point_step=24).
 * @param cloud Output message; must be non-null (cleared fields assumed empty).
 * @param frame_id Header frame_id.
 */
inline void InitXyzitCloud(automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                           const std::string& frame_id) {
    cloud->mutable_header()->set_frame_id(frame_id);
    cloud->set_height(1);
    cloud->set_is_dense(false);
    cloud->set_is_bigendian(false);
    using PF = automsgs::msgs::sensor_msgs::PointField;
    AddPointField(cloud, "x", 0, PF::FLOAT32);
    AddPointField(cloud, "y", 4, PF::FLOAT32);
    AddPointField(cloud, "z", 8, PF::FLOAT32);
    AddPointField(cloud, "intensity", 12, PF::FLOAT32);
    AddPointField(cloud, "timestamp", 16, PF::FLOAT64);
    cloud->set_point_step(kXyzitPointStep);
    cloud->set_row_step(0);
    cloud->set_width(0);
}

/**
 * @brief Append one XYZIT point to the packed @p data buffer.
 * @param data PointCloud2 mutable data string; must be non-null.
 * @param x X in metres.
 * @param y Y in metres.
 * @param z Z in metres.
 * @param intensity Intensity / reflectivity.
 * @param timestamp_ns Point timestamp in nanoseconds (stored as float64).
 */
inline void AppendXyzitPoint(std::string* data, float x, float y, float z,
                             float intensity, double timestamp_ns) {
    char point[kXyzitPointStep];
    std::memcpy(point + 0, &x, 4);
    std::memcpy(point + 4, &y, 4);
    std::memcpy(point + 8, &z, 4);
    std::memcpy(point + 12, &intensity, 4);
    std::memcpy(point + 16, &timestamp_ns, 8);
    data->append(point, kXyzitPointStep);
}

/**
 * @brief Set width and row_step after all points were appended.
 * @param cloud Target PointCloud2; must be non-null.
 * @param width Number of points written into data.
 */
inline void FinishXyzitCloud(automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                             std::uint32_t width) {
    cloud->set_width(width);
    cloud->set_row_step(cloud->point_step() * width);
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_POINT_CLOUD2_LAYOUT_HPP_
