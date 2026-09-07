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
 * @brief Velodyne Scan packets → PointCloud2 (VLP-16 geometry).
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_CONVERT_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_CONVERT_HPP_

#include <string>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/lidar/velodyne/calibration.hpp"
#include "autodriver/lidar/velodyne/packet.hpp"

namespace autodriver {
namespace lidar {
namespace velodyne {

/**
 * @brief Converts packets using optional beam calibration (vert radians).
 * Empty calibration → DefaultVlp16Calibration(). Unknown model → VLP-16 + warn
 * is handled by the caller; this function uses `calibration` only.
 */
automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const BeamCalibration& calibration);

/**
 * @brief Convenience: model "VLP-16" (default) or any string with built-in table.
 */
automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id = "velodyne",
    const std::string& model = "VLP-16");

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_CONVERT_HPP_
