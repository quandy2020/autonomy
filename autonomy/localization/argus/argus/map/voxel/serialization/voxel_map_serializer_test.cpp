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

#include "map/voxel/serialization/voxel_map_serializer.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <vector>

namespace argus {
namespace vxslam {
namespace {

TEST(VoxelMapSerializer, ParsesPreviousMapListAndRoundTripsTrajectory) {
  const std::vector<PreviousMapSpec> specs =
      ParsePreviousMapList("site1: 0.50, site2: 0.45");
  ASSERT_EQ(specs.size(), 2u);
  EXPECT_EQ(specs[0].name, "site1");

  std::vector<ImuState> poses(2);
  poses[0].t = 1.0;
  poses[0].p = Eigen::Vector3d(1.0, 2.0, 3.0);
  poses[1].t = 2.0;
  poses[1].p = Eigen::Vector3d(4.0, 5.0, 6.0);

  const char* path = "/tmp/argus_vxslam_traj_test.tum";
  ASSERT_TRUE(SaveTrajectoryTum(path, poses));
  std::vector<ImuState> loaded;
  ASSERT_TRUE(LoadTrajectoryTum(path, &loaded));
  ASSERT_EQ(loaded.size(), 2u);
  EXPECT_NEAR(loaded[1].p.x(), 4.0, 1e-6);
  std::remove(path);
}

}  // namespace
}  // namespace vxslam
}  // namespace argus
