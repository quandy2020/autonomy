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
 * @brief Rigid transform, z-crop, and invalid-point removal for LIVO clouds.
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_

#include <Eigen/Geometry>

#include "sensor/point_cloud_data.hpp"

namespace argus {
namespace sensor {

/**
 * @brief Apply a rigid transform to XYZ; rotate normals; keep intensity/time.
 * @param[in] cloud     Input cloud in the source frame.
 * @param[in] transform Target ← source isometry (@c Eigen::Isometry3f).
 * @return Transformed cloud copy (same size as @p cloud).
 */
PointCloudXYZI TransformPointCloud(const PointCloudXYZI& cloud,
                                   const Eigen::Isometry3f& transform);

/**
 * @brief Keep points whose z lies in [@p min_z, @p max_z] (sensor frame).
 * @param[in] cloud Input cloud.
 * @param[in] min_z Minimum z in metres (inclusive).
 * @param[in] max_z Maximum z in metres (inclusive).
 * @return Filtered cloud copy; order of surviving points preserved.
 */
PointCloudXYZI CropPointCloud(const PointCloudXYZI& cloud, float min_z,
                              float max_z);

/**
 * @brief Drop non-finite coordinates or exact (0, 0, 0) points.
 * @param[in] cloud Input cloud.
 * @return Cloud with invalid entries removed; remaining points unchanged.
 */
PointCloudXYZI RemoveInvalidPoints(const PointCloudXYZI& cloud);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_TRANSFORM_HPP_
