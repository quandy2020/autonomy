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

#include "backend/global_mapping/pgo_edges.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace argus {
namespace vxslam {
namespace {

TEST(PgoEdges, PushAndBuildPoseGraph) {
  PgoEdges edges;
  Eigen::Matrix<double, 6, 1> information;
  information.setConstant(1e-6);
  edges.Push(0, 1, 0, 5, Eigen::Matrix3d::Identity(),
             Eigen::Vector3d(1.0, 0.0, 0.0), information);
  ASSERT_EQ(edges.edges.size(), 1u);
  EXPECT_EQ(edges.edges[0].ids1[0], 0);

  std::vector<ImuState> poses(3);
  for (int i = 0; i < 3; ++i) {
    poses[static_cast<std::size_t>(i)].p = Eigen::Vector3d(i, 0.0, 0.0);
  }
  backend::PoseGraph graph;
  BuildPoseGraph(&graph, poses, {});
  const backend::OptimizeResult result = graph.Optimize(5);
  EXPECT_TRUE(result.ok);
}

}  // namespace
}  // namespace vxslam
}  // namespace argus
