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
 * @brief Hash-voxel plane map for lidar scan-to-map.
 *
 * Points are binned by `voxel_size`. `Commit` fits a plane when the smallest
 * covariance eigenvalue is below `min_eigenvalue`, and subdivides otherwise.
 * Point variance and the slide-window marginalization of Voxel-SLAM are later.
 */

#ifndef ARGUS_MAP_VOXEL_PLANE_MAP_HPP_
#define ARGUS_MAP_VOXEL_PLANE_MAP_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <memory>

namespace argus {
namespace map {
namespace voxel {

class VoxelPlaneMap {
 public:
  struct Config {
    double voxel_size = 1.0;
    double min_eigenvalue = 0.01;
    double plane_ratio = 1.0;
    double max_plane_distance = 0.1;
    int min_points = 5;
    int max_layer = 2;
  };

  VoxelPlaneMap();
  explicit VoxelPlaneMap(Config config);
  ~VoxelPlaneMap();

  VoxelPlaneMap(VoxelPlaneMap&&) noexcept;
  VoxelPlaneMap& operator=(VoxelPlaneMap&&) noexcept;

  VoxelPlaneMap(const VoxelPlaneMap&) = delete;
  VoxelPlaneMap& operator=(const VoxelPlaneMap&) = delete;

  void Insert(const Eigen::Vector3d& point);
  void Commit();

  bool Query(const Eigen::Vector3d& point, Eigen::Vector3d* center,
             Eigen::Vector3d* normal) const;
  std::size_t plane_count() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_PLANE_MAP_HPP_
