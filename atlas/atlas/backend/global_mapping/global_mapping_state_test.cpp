/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/global_mapping/global_mapping_state.hpp"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

void MakeSingleKeyframeRequest(GlobalMappingRequest* request,
                               GlobalMappingResult* result) {
  request->request_id = 1;
  request->source_pose_version = 2;
  request->source_map_state_version = 3;
  request->session_id = 4;
  request->keyframe_indices = {0};
  request->initial_poses.resize(1);
  request->keyframes = {{7, 4, nullptr}};

  result->request_id = request->request_id;
  result->source_pose_version = request->source_pose_version;
  result->source_map_state_version = request->source_map_state_version;
  result->source_session_id = request->session_id;
  result->status = GlobalMappingStatus::kSucceeded;
  result->keyframe_indices = request->keyframe_indices;
  result->optimized_poses.resize(1);
  result->optimized_poses.front().p.x() = 5.0;
}

TEST(GlobalMappingState, CommitsLocalWindowAndAdvancesCursor) {
  GlobalMappingState state;
  GlobalMappingRequest request;
  GlobalMappingResult result;
  MakeSingleKeyframeRequest(&request, &result);
  std::vector<std::unique_ptr<map::Keyframe>> keyframes;
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes.front()->keyframe_id = 7;
  keyframes.front()->session_id = 4;
  auto submap = std::make_unique<map::Keyframe>(InertialState{});
  std::uint64_t pose_version = 2;
  std::uint64_t map_version = 3;

  EXPECT_TRUE(state.CommitLocalWindow(request, &result, &pose_version,
                                      &map_version, 4, keyframes,
                                      std::move(submap), 1));

  EXPECT_EQ(state.next_keyframe_index(), 1U);
  ASSERT_EQ(state.submaps().size(), 1U);
  EXPECT_DOUBLE_EQ(keyframes.front()->inertial_state.p.x(), 5.0);
  EXPECT_EQ(pose_version, 3U);
  EXPECT_EQ(map_version, 4U);
}

TEST(GlobalMappingState, RejectsStaleLocalResultWithoutAdvancingState) {
  GlobalMappingState state;
  GlobalMappingRequest request;
  GlobalMappingResult result;
  MakeSingleKeyframeRequest(&request, &result);
  std::vector<std::unique_ptr<map::Keyframe>> keyframes;
  keyframes.push_back(std::make_unique<map::Keyframe>(InertialState{}));
  keyframes.front()->keyframe_id = 7;
  keyframes.front()->session_id = 4;
  auto submap = std::make_unique<map::Keyframe>(InertialState{});
  std::uint64_t pose_version = 3;
  std::uint64_t map_version = 3;

  EXPECT_FALSE(state.CommitLocalWindow(request, &result, &pose_version,
                                       &map_version, 4, keyframes,
                                       std::move(submap), 1));

  EXPECT_EQ(state.next_keyframe_index(), 0U);
  EXPECT_TRUE(state.submaps().empty());
  EXPECT_DOUBLE_EQ(keyframes.front()->inertial_state.p.x(), 0.0);
  EXPECT_EQ(pose_version, 3U);
  EXPECT_EQ(map_version, 3U);
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
