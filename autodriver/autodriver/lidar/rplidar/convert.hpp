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
 * @brief Convert RPLidar HQ nodes to sensor_msgs/LaserScan (rplidar_ros aligned).
 */

#ifndef AUTODRIVER_LIDAR_RPLIDAR_CONVERT_HPP_
#define AUTODRIVER_LIDAR_RPLIDAR_CONVERT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>

#ifdef AUTODRIVER_HAVE_RPLIDAR
#include <sl_lidar.h>
#endif

namespace autodriver {
namespace lidar {
namespace rplidar {

/**
 * @brief Options for NodesToLaserScan (rplidar_ros publish_scan aligned).
 */
struct ConvertOptions {
    std::string frame_id{"laser"};  ///< LaserScan header frame_id.
    bool inverted{false};           ///< Flip scan direction when true.
    float range_min{0.15f};         ///< Metres; ranges below discarded.
    float range_max{12.0f};         ///< Metres; ranges above discarded.
    double scan_time_s{0.1};        ///< Time between scans (seconds).
};

#ifdef AUTODRIVER_HAVE_RPLIDAR
/**
 * @brief Fill LaserScan from HQ nodes (aligned with rplidar_ros publish_scan).
 * @param nodes Contiguous HQ measurement nodes from the SDK.
 * @param count Number of nodes in @p nodes.
 * @param angle_min_rad Inclusive start angle (radians).
 * @param angle_max_rad Inclusive end angle (radians).
 * @param opt Frame id, invert, range limits, and scan_time.
 * @return Filled LaserScan message (may have empty ranges on bad input).
 */
automsgs::msgs::sensor_msgs::LaserScan NodesToLaserScan(
    const sl_lidar_response_measurement_node_hq_t* nodes, std::size_t count,
    float angle_min_rad, float angle_max_rad, const ConvertOptions& opt);
#endif

}  // namespace rplidar
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_RPLIDAR_CONVERT_HPP_
