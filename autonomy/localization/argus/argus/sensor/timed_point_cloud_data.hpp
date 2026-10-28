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
 * @file timed_point_cloud_data.hpp
 * @brief One synchronized ranging scan.
 *
 * `time` is the acquisition time of the last return, in seconds.
 * Each point's `time` is relative to that instant and is therefore <= 0.
 */

#ifndef ARGUS_SENSOR_TIMED_POINT_CLOUD_DATA_HPP_
#define ARGUS_SENSOR_TIMED_POINT_CLOUD_DATA_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "sensor/point_cloud.hpp"

namespace argus {
namespace sensor {

struct TimedPointCloudData {
  double time = 0.0;
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  TimedPointCloud ranges;
  std::vector<float> intensities;
};

struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    float intensity = 0.f;
    size_t origin_index = 0;
  };
  double time = 0.0;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_TIMED_POINT_CLOUD_DATA_HPP_
