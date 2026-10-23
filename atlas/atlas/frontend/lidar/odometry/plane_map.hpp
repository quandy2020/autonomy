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
 * @file plane_map.hpp
 * @brief Read-only plane query used by lidar odometry.
 *
 * `VoxelPlaneQuery` adapts `map::voxel::VoxelPlaneMap` to this interface.
 */

#ifndef ATLAS_FRONTEND_LIDAR_PLANE_MAP_HPP_
#define ATLAS_FRONTEND_LIDAR_PLANE_MAP_HPP_

#include <Eigen/Core>

namespace atlas {
namespace frontend {
namespace lidar {

struct Plane {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

class PlaneMap {
 public:
  virtual ~PlaneMap() = default;
  virtual bool Query(const Eigen::Vector3d& point, Plane* plane) const = 0;
};

}  // namespace lidar
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_LIDAR_PLANE_MAP_HPP_
