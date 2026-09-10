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

/**
 * @file
 * @brief Livox points → sensor_msgs/PointCloud2 (x,y,z,intensity,timestamp).
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_CONVERT_HPP_
#define AUTODRIVER_LIDAR_LIVOX_CONVERT_HPP_

#include <string>
#include <vector>

#include "autodriver/lidar/livox/points.hpp"
#include "automsgs/msgs/sensor_msgs/point_cloud2.pb.h"

namespace autodriver {
namespace lidar {
namespace livox {

/**
 * @brief Build PointCloud2 with shared XYZIT layout (point_step=24).
 * @param points XYZIT points in the lidar frame.
 * @param frame_id Header frame_id written into the message.
 * @return PointCloud2 via InitXyzitCloud / AppendXyzitPoint; empty when no points.
 */
automsgs::msgs::sensor_msgs::PointCloud2 PointsToPointCloud(
    const std::vector<PointXYZIT>& points, const std::string& frame_id);

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_CONVERT_HPP_
