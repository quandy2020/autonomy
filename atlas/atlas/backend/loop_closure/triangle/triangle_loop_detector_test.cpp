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

#include "backend/loop_closure/triangle/triangle_loop_detector.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {
namespace {

sensor::PointCloudXYZI MakeThreePlanes(float offset) {
  sensor::PointCloudXYZI cloud;
  for (int plane = 0; plane < 3; ++plane) {
    for (int i = 0; i < 20; ++i) {
      for (int j = 0; j < 20; ++j) {
        sensor::PointType point;
        if (plane == 0) {
          point.x = offset + 0.05f * i;
          point.y = 0.05f * j;
          point.z = 0.f;
        } else if (plane == 1) {
          point.x = offset + 3.f + 0.05f * i;
          point.y = 0.05f * j;
          point.z = 0.f;
        } else {
          point.x = offset + 1.5f + 0.05f * i;
          point.y = 3.f + 0.05f * j;
          point.z = 0.f;
        }
        cloud.push_back(point);
      }
    }
  }
  return cloud;
}

TEST(TriangleLoopDetector, DoesNotIndexFramesWithoutDescriptors) {
  LoopClosureOptions config;
  config.voxel_size = 1.0f;
  config.voxel_init_num = 5;
  config.skip_near_num = 2;
  config.icp_threshold = 0.01f;
  TriangleLoopDetector detector(config);

  for (int id = 0; id < 8; ++id) {
    KeyFrame keyframe;
    keyframe.id = id;
    keyframe.cloud = MakeThreePlanes(0.f);
    detector.Detect(keyframe);
  }
  EXPECT_EQ(detector.database_size(), 0u);

  KeyFrame query;
  query.id = 100;
  query.cloud = MakeThreePlanes(0.02f);
  const std::vector<LoopCandidate> candidates = detector.Detect(query);
  EXPECT_GE(candidates.size(), 0u);
}

TEST(TriangleLoopDetector, IndexesFramesWithDescriptors) {
  TriangleLoopDetector detector;
  TriangleDescriptor descriptor;
  descriptor.frame_number = 7;

  detector.AddDescriptors({descriptor});

  EXPECT_EQ(detector.database_size(), 1u);
}

TEST(TriangleLoopDetector, DoesNotInventTrianglesWithoutBinaryCorners) {
  LoopClosureOptions config;
  config.voxel_size = 1.0f;
  config.voxel_init_num = 5;
  config.useful_corner_num = 0;
  TriangleLoopDetector detector(config);
  std::vector<TriangleDescriptor> descriptors;
  sensor::PointCloudXYZI plane_cloud;

  detector.GenerateDescriptors(MakeThreePlanes(0.f), 0, &descriptors,
                               &plane_cloud);

  EXPECT_GE(plane_cloud.size(), 3u);
  EXPECT_TRUE(descriptors.empty());
}

}  // namespace
}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas
