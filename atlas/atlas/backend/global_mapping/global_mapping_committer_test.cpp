/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/global_mapping/global_mapping_committer.hpp"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

TEST(GlobalMappingCommitter,
     RejectsMismatchedLaterKeyframeWithoutPartialPoseCommit) {
  GlobalMappingRequest request;
  request.request_id = 1;
  request.source_pose_version = 2;
  request.source_map_state_version = 3;
  request.session_id = 4;
  request.keyframe_indices = {0, 1};
  request.initial_poses.resize(2);
  request.keyframes = {{10, 4, nullptr}, {11, 4, nullptr}};

  GlobalMappingResult result;
  result.request_id = request.request_id;
  result.source_pose_version = request.source_pose_version;
  result.source_map_state_version = request.source_map_state_version;
  result.source_session_id = request.session_id;
  result.status = GlobalMappingStatus::kSucceeded;
  result.keyframe_indices = request.keyframe_indices;
  result.optimized_poses.resize(2);
  result.optimized_poses[0].p.x() = 5.0;
  result.optimized_poses[1].p.x() = 6.0;

  std::vector<std::unique_ptr<map::Keyframe>> keyframes;
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes[0]->keyframe_id = 10;
  keyframes[0]->session_id = 4;
  keyframes[1]->keyframe_id = 99;
  keyframes[1]->session_id = 4;

  GlobalMappingCommitter committer;
  EXPECT_FALSE(committer.Commit(request, result, 2, 3, 4, keyframes));
  EXPECT_DOUBLE_EQ(keyframes[0]->inertial_state.p.x(), 0.0);
  EXPECT_DOUBLE_EQ(keyframes[1]->inertial_state.p.x(), 0.0);
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
