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
 * @brief Standardized 2-D LiDAR scan for HAL consumers.
 */

#ifndef AUTODRIVER_TYPES_LIDAR_SCAN_HPP_
#define AUTODRIVER_TYPES_LIDAR_SCAN_HPP_

#include <memory>
#include <vector>

#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @struct LidarScan
 * @brief Planar range scan with per-beam angles.
 */
struct LidarScan : public SensorSample
{
  /** Range readings [m]; NaN marks invalid returns. */
  std::vector<float> ranges;
  /** Beam angles [rad] relative to the sensor x-axis. */
  std::vector<float> angles;
  float range_min{0.0f};
  float range_max{0.0f};

  LidarScan(
    SensorId sensor_id,
    Timestamp device_time,
    std::vector<float> ranges,
    std::vector<float> angles,
    float range_min,
    float range_max)
  : SensorSample(std::move(sensor_id), SensorType::kLidar, device_time),
    ranges(std::move(ranges)),
    angles(std::move(angles)),
    range_min(range_min),
    range_max(range_max)
  {
  }

  std::unique_ptr<SensorSample> Clone() const override
  {
    return std::make_unique<LidarScan>(*this);
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_LIDAR_SCAN_HPP_
