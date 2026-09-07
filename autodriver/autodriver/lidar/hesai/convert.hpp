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
 * @brief Hesai Scan packets → PointCloud2 (PandarXT-32 geometry).
 */

#ifndef AUTODRIVER_LIDAR_HESAI_CONVERT_HPP_
#define AUTODRIVER_LIDAR_HESAI_CONVERT_HPP_

#include <array>
#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/lidar/hesai/packet.hpp"
#include "autodriver/lidar/hesai/calibration.hpp"

namespace autodriver {
namespace lidar {
namespace hesai {

/**
 * @brief Design vertical angles (deg) for PandarXT / XT32 channels 1..32.
 * Manual Appendix I; upward positive, channel 1 = +15°.
 */
std::array<double, kChannelsPerBlock> DefaultXt32VerticalAnglesDeg();

/**
 * @brief Convert XT32 UDP packets to PointCloud2 (x,y,z,intensity,timestamp).
 */
automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const BeamCalibration& calibration);

/**
 * @brief Convert using built-in XT32 elevations (or warn for unknown model).
 */
automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id = "hesai",
    const std::string& model = "XT32");

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_CONVERT_HPP_
