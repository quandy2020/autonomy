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
 * @file range_data.hpp
 * @brief Returns and misses of one scan, expressed from a common origin.
 */

#ifndef ARGUS_SENSOR_RANGE_DATA_HPP_
#define ARGUS_SENSOR_RANGE_DATA_HPP_

#include <Eigen/Core>

#include "common/transform/rigid_transform.hpp"
#include "sensor/point_cloud.hpp"

namespace argus {
namespace sensor {

struct RangeData {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  PointCloud returns;
  PointCloud misses;
};

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_RANGE_DATA_HPP_
