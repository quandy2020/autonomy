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

#include "map/voxel/hierarchical_bundle_adjustment.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "map/voxel/global_bundle_adjustment_octree.hpp"

namespace argus {
namespace vxslam {
namespace {

TEST(HierarchicalBundleAdjustment, OptimizeReturnsFiniteResidualOnThreeFrames) {
  std::vector<ImuState> poses(3);
  std::vector<sensor::PointCloudXYZI> clouds(3);
  for (int i = 0; i < 3; ++i) {
    poses[static_cast<std::size_t>(i)].p = Eigen::Vector3d(i * 0.5, 0.0, 0.0);
    for (int u = 0; u < 15; ++u) {
      for (int v = 0; v < 15; ++v) {
        sensor::PointType point;
        point.x = 0.1f * u;
        point.y = 0.1f * v;
        point.z = 0.0f;
        clouds[static_cast<std::size_t>(i)].push_back(point);
      }
    }
  }
  HierarchicalBundleAdjustment gba;
  const double residual = gba.Optimize(&poses, clouds);
  EXPECT_TRUE(std::isfinite(residual));
}

}  // namespace
}  // namespace vxslam
}  // namespace argus

namespace argus::vxslam {
namespace {

TEST(HierarchicalBundleAdjustment, VoxelIndexUsesFloorAtNegativeBoundaries) {
  auto configuration =
      std::make_shared<const GlobalBundleAdjustmentOctreeConfiguration>();
  std::unordered_map<map::voxel::VoxelKey,
                     std::unique_ptr<GlobalBundleAdjustmentOctree>>
      voxel_octrees;
  sensor::PointCloudXYZI cloud;
  sensor::PointType point;
  point.x = -1.0f;
  point.y = 0.0f;
  point.z = 0.0f;
  cloud.push_back(point);
  GlobalBundleAdjustmentOctree::InsertPointCloudIntoVoxelMap(
      &voxel_octrees, ImuState{}, cloud, 0, 1, configuration);
  EXPECT_EQ(voxel_octrees.count(map::voxel::VoxelKey(-1, 0, 0)), 1u);
}

TEST(HierarchicalBundleAdjustment, BuildsPlaneWithMoreWorkersThanVoxels) {
  auto configuration =
      std::make_shared<const GlobalBundleAdjustmentOctreeConfiguration>();
  std::unordered_map<map::voxel::VoxelKey,
                     std::unique_ptr<GlobalBundleAdjustmentOctree>>
      voxel_octrees;
  sensor::PointCloudXYZI cloud;
  for (int point_index = 0; point_index < 16; ++point_index) {
    sensor::PointType point;
    point.x = 0.1f * (point_index % 4);
    point.y = 0.1f * (point_index / 4);
    point.z = 0.0f;
    cloud.push_back(point);
  }
  for (int window_index = 0; window_index < 2; ++window_index) {
    GlobalBundleAdjustmentOctree::InsertPointCloudIntoVoxelMap(
        &voxel_octrees, ImuState{}, cloud, window_index, 2, configuration);
  }
  common::ThreadPool pool(2);
  LidarFactor factors(2);
  RebuildVoxelPlaneFactors(&voxel_octrees, &factors, 4, &pool);
  EXPECT_TRUE(voxel_octrees.empty());
  EXPECT_EQ(factors.plvec_voxels.size(), 1u);
}

}  // namespace
}  // namespace argus::vxslam
