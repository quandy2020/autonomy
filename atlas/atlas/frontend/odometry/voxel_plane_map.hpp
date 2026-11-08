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
 * @file voxel_plane_map.hpp
 * @brief `PlaneMap` view of `::atlas::map::VoxelPlaneMap`.
 */

#ifndef ATLAS_FRONTEND_ODOMETRY_VOXEL_PLANE_MAP_HPP_
#define ATLAS_FRONTEND_ODOMETRY_VOXEL_PLANE_MAP_HPP_

#include "frontend/odometry/plane_map.hpp"
#include "map/voxel_plane_map.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

class VoxelPlaneQuery : public PlaneMap {
 public:
  explicit VoxelPlaneQuery(const ::atlas::map::VoxelPlaneMap& map)
      : map_(&map) {}

  bool Query(const Eigen::Vector3d& point,
             PlaneEstimate* plane) const override {
    if (plane == nullptr || map_ == nullptr) {
      return false;
    }
    return map_->Query(point, &plane->center, &plane->normal);
  }

 private:
  const ::atlas::map::VoxelPlaneMap* map_ = nullptr;
};

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_ODOMETRY_VOXEL_PLANE_MAP_HPP_
