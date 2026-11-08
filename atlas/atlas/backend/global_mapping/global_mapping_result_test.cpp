/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/global_mapping/global_mapping_result.hpp"

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

GlobalMappingResult MakeMatchingResult(const GlobalMappingRequest& request) {
  GlobalMappingResult result;
  result.request_id = request.request_id;
  result.source_pose_version = request.source_pose_version;
  result.source_map_state_version = request.source_map_state_version;
  result.source_session_id = request.session_id;
  result.keyframe_indices = request.keyframe_indices;
  return result;
}

TEST(GlobalMappingResult, AcceptsSuccessfulResultForMatchingVersions) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;

  EXPECT_TRUE(result.MatchesRequest(request));
}

TEST(GlobalMappingResult, AcceptsResultForCurrentSourceState) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;

  EXPECT_TRUE(result.MatchesCurrentState(request, 23, 29, 31));
}

TEST(GlobalMappingResult, RejectsResultAfterSourcePoseChanges) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;

  EXPECT_FALSE(result.MatchesCurrentState(request, 24, 29, 31));
}

TEST(GlobalMappingResult, RejectsResultAfterMapStateChanges) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;

  EXPECT_FALSE(result.MatchesCurrentState(request, 23, 30, 31));
}

TEST(GlobalMappingResult, RejectsResultAfterSessionChanges) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;

  EXPECT_FALSE(result.MatchesCurrentState(request, 23, 29, 32));
}

TEST(GlobalMappingResult, RejectsResultFromDifferentPoseVersion) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;
  ++result.source_pose_version;

  EXPECT_FALSE(result.MatchesRequest(request));
}

TEST(GlobalMappingResult, RejectsResultFromDifferentMapVersion) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kSucceeded;
  ++result.source_map_state_version;

  EXPECT_FALSE(result.MatchesRequest(request));
}

TEST(GlobalMappingResult, RejectsUnsuccessfulResult) {
  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 23;
  request.source_map_state_version = 29;
  request.session_id = 31;
  GlobalMappingResult result = MakeMatchingResult(request);
  result.status = GlobalMappingStatus::kFailed;

  EXPECT_FALSE(result.MatchesRequest(request));
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
