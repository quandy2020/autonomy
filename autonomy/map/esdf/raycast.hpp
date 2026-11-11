/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file raycast.hpp
 * @brief Amanatides–Woo voxel traversal (FIESTA raycast, no ROS).
 */

#pragma once

#include <vector>

#include <Eigen/Core>

namespace autonomy {
namespace map {
namespace esdf {

/**
 * @brief Walk voxels from @p start to @p end in grid coordinates.
 *
 * Coordinates are typically world metres divided by resolution. Only voxels
 * whose integer indices lie in [@p min_bound, @p max_bound) are appended.
 * A zero-length step in any axis uses infinite parametric time so that axis
 * is skipped. If start and end fall in the same voxel, @p output is cleared
 * and the function returns without appending.
 *
 * @param[in] start Ray origin in grid coordinates (world / resolution).
 * @param[in] end Ray endpoint in grid coordinates (world / resolution).
 * @param[in] min_bound Inclusive AABB minimum in grid coordinates. Voxels
 *                      with index < min_bound on any axis are skipped.
 * @param[in] max_bound Exclusive AABB maximum in grid coordinates. Voxels
 *                      with index >= max_bound on any axis are skipped.
 * @param[out] output Destination list. Cleared, then filled with visited
 *                    voxel centres as Vector3d(voxel_x, voxel_y, voxel_z).
 *                    Must not be null.
 * @throw std::out_of_range when more than 1500 in-bounds voxels are visited.
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound,
             std::vector<Eigen::Vector3d>* output);

}  // namespace esdf
}  // namespace map
}  // namespace autonomy
