/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autodriver/lidar/livox/convert.hpp"

#include <cstring>

namespace autodriver {
namespace lidar {
namespace livox {
namespace {

void AddPointField(automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                   const std::string& name, std::uint32_t offset,
                   automsgs::msgs::sensor_msgs::PointField::DataType datatype,
                   std::uint32_t count = 1) {
    auto* field = cloud->add_fields();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(datatype);
    field->set_count(count);
}

}  // namespace

automsgs::msgs::sensor_msgs::PointCloud2 PointsToPointCloud(
    const std::vector<PointXYZIT>& points, const std::string& frame_id) {
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id(frame_id);
    cloud.set_height(1);
    cloud.set_is_dense(false);
    cloud.set_is_bigendian(false);
    using PF = automsgs::msgs::sensor_msgs::PointField;
    AddPointField(&cloud, "x", 0, PF::FLOAT32);
    AddPointField(&cloud, "y", 4, PF::FLOAT32);
    AddPointField(&cloud, "z", 8, PF::FLOAT32);
    AddPointField(&cloud, "intensity", 12, PF::FLOAT32);
    AddPointField(&cloud, "timestamp", 16, PF::FLOAT64);
    cloud.set_point_step(24);

    std::string& data = *cloud.mutable_data();
    data.reserve(points.size() * 24);
    for (const PointXYZIT& p : points) {
        char buf[24];
        std::memcpy(buf + 0, &p.x, 4);
        std::memcpy(buf + 4, &p.y, 4);
        std::memcpy(buf + 8, &p.z, 4);
        std::memcpy(buf + 12, &p.intensity, 4);
        std::memcpy(buf + 16, &p.timestamp_ns, 8);
        data.append(buf, 24);
    }
    cloud.set_width(static_cast<std::uint32_t>(points.size()));
    cloud.set_row_step(cloud.point_step() * cloud.width());
    return cloud;
}

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver
