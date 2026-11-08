/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/global_mapping/global_mapping_request_builder.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

TEST(GlobalMappingRequestBuilder,
     BuildsLocalWindowSnapshotFromPendingKeyframes) {
  std::vector<std::unique_ptr<map::Keyframe>> keyframes;
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes.push_back(nullptr);
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes[0]->keyframe_id = 5;
  keyframes[0]->session_id = 3;
  keyframes[2]->keyframe_id = 7;
  keyframes[2]->session_id = 3;
  keyframes[2]->point_cloud = std::make_shared<const sensor::PointCloudXYZI>();

  GlobalMappingRequest metadata;
  metadata.request_id = 11;
  metadata.source_pose_version = 13;
  metadata.source_map_state_version = 17;
  metadata.session_id = 3;
  metadata.maximum_iteration_count = 1;
  metadata.worker_thread_count = 2;

  const GlobalMappingRequest request =
      GlobalMappingRequestBuilder().BuildLocalWindowRequest(metadata, 1,
                                                            2, keyframes);

  EXPECT_EQ(request.request_id, 11);
  EXPECT_EQ(request.source_pose_version, 13);
  EXPECT_EQ(request.source_map_state_version, 17);
  EXPECT_EQ(request.keyframe_indices, (std::vector<std::size_t>{2}));
  ASSERT_EQ(request.keyframes.size(), 1U);
  EXPECT_EQ(request.keyframes.front().keyframe_id, 7);
  EXPECT_EQ(request.keyframes.front().session_id, 3);
  EXPECT_EQ(request.keyframes.front().cloud, keyframes[2]->point_cloud);
  ASSERT_EQ(request.initial_poses.size(), 1U);
  EXPECT_DOUBLE_EQ(request.initial_poses.front().t,
                   keyframes[2]->inertial_state.t);
}

TEST(GlobalMappingRequestBuilder, BuildsSubmapSnapshotWithOriginalIndices) {
  std::vector<std::unique_ptr<map::Keyframe>> submaps;
  submaps.push_back(nullptr);
  submaps.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  submaps[1]->keyframe_id = 21;
  submaps[1]->session_id = 8;

  GlobalMappingRequest metadata;
  metadata.request_id = 4;
  metadata.session_id = 8;

  const GlobalMappingRequest request =
      GlobalMappingRequestBuilder().BuildSubmapRequest(metadata, submaps);

  EXPECT_EQ(request.request_id, 4);
  EXPECT_EQ(request.keyframe_indices, (std::vector<std::size_t>{1}));
  ASSERT_EQ(request.keyframes.size(), 1U);
  EXPECT_EQ(request.keyframes.front().keyframe_id, 21);
}

TEST(GlobalMappingRequestBuilder, RejectsCursorBeyondKeyframeStorage) {
  std::vector<std::unique_ptr<map::Keyframe>> keyframes;

  EXPECT_THROW(GlobalMappingRequestBuilder().BuildLocalWindowRequest(
                   GlobalMappingRequest{}, 1, 10, keyframes),
               std::logic_error);
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
