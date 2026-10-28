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

#include <gtest/gtest.h>

#include <vector>

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_detector.hpp"
#include "frontend/lidar/odometry/scan_pose.hpp"
#include "map/voxel/hierarchical_bundle_adjustment.hpp"

namespace argus {
namespace vxslam {
namespace {

TEST(ScanPose, UpdateAppliesLeftPerturbation) {
  ScanPose pose;
  pose.x.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  ImuState delta;
  delta.p = Eigen::Vector3d(1.0, 0.0, 0.0);
  pose.Update(delta);
  EXPECT_NEAR(pose.x.p.x(), 2.0, 1e-12);
}

TEST(HierarchicalBundleAdjustment, EmptyInputIsSafe) {
  HierarchicalBundleAdjustment gba;
  std::vector<ImuState> states;
  std::vector<Keyframe*> submaps;
  PgoEdges edges;
  gba.AddEdgesFromHierarchicalBundleAdjustment(&states, submaps, &edges, {0}, 1,
                                               1);
  EXPECT_TRUE(edges.edges.empty());
}

}  // namespace
}  // namespace vxslam
}  // namespace argus
