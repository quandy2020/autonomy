/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file convert.cpp
 * @brief Livox points → sensor_msgs/PointCloud2 (x,y,z,intensity,timestamp)
 *        (implementation).
 */

#include "autodriver/lidar/livox/convert.hpp"

#include "autodriver/lidar/point_cloud2_layout.hpp"

namespace autodriver {
namespace lidar {
namespace livox {

automsgs::msgs::sensor_msgs::PointCloud2 PointsToPointCloud(
    const std::vector<PointXYZIT>& points, const std::string& frame_id) {
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    InitXyzitCloud(&cloud, frame_id);

    std::string& data = *cloud.mutable_data();
    data.reserve(points.size() * kXyzitPointStep);
    for (const PointXYZIT& p : points) {
        AppendXyzitPoint(&data, p.x, p.y, p.z, p.intensity, p.timestamp_ns);
    }
    FinishXyzitCloud(&cloud, static_cast<std::uint32_t>(points.size()));
    return cloud;
}

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver
