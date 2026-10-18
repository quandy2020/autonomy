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
 * @file point_cloud_transform.hpp
 * @brief LIVO point-cloud transform / crop helpers.
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_

#include <Eigen/Geometry>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace sensor {

/**
 * @brief Apply rigid transform to XYZ; rotates normals; keeps intensity/time.
 * @param cloud Input cloud in source frame.
 * @param transform Target ← source isometry.
 * @return Transformed cloud copy.
 */
PointCloudXYZI TransformPointCloud(const PointCloudXYZI& cloud,
                                   const Eigen::Isometry3f& transform);

/**
 * @brief Keep points with z in [min_z, max_z] (sensor frame).
 * @param cloud Input cloud.
 * @param min_z Minimum z (m).
 * @param max_z Maximum z (m).
 * @return Filtered cloud copy.
 */
PointCloudXYZI CropPointCloud(const PointCloudXYZI& cloud, float min_z,
                              float max_z);

/**
 * @brief Drop non-finite or (0,0,0) points.
 * @param cloud Input cloud.
 * @return Cloud with invalid entries removed.
 */
PointCloudXYZI RemoveInvalidPoints(const PointCloudXYZI& cloud);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_
