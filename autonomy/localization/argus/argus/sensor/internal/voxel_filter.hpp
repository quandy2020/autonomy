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
 * @file voxel_filter.hpp
 * @brief Voxel downsampling. One point is kept per cell, chosen by reservoir
 *        sampling so repeated calls on the same cloud are deterministic.
 */

#ifndef ARGUS_SENSOR_INTERNAL_VOXEL_FILTER_HPP_
#define ARGUS_SENSOR_INTERNAL_VOXEL_FILTER_HPP_

#include <vector>

#include "sensor/point_cloud.hpp"
#include "sensor/timed_point_cloud_data.hpp"

namespace argus {
namespace sensor {

struct AdaptiveVoxelFilterOptions {
  float max_length = 0.5f;
  float min_num_points = 100.f;
  float max_range = 50.f;
};

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, float resolution);
PointCloud VoxelFilter(const PointCloud& point_cloud, float resolution);
TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            float resolution);
std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    float resolution);

PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
                               const AdaptiveVoxelFilterOptions& options);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_INTERNAL_VOXEL_FILTER_HPP_
