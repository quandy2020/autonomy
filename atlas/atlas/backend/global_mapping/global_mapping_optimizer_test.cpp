/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping_optimizer.hpp"

#include <memory>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

TEST(GlobalMappingOptimizer, ReturnsSuccessfulValidatedWindowResult) {
  ::atlas::map::GlobalBundleAdjustmentConfiguration configuration;
  configuration.worker_thread_count = 1;
  configuration.maximum_iteration_count = 1;
  GlobalMappingOptimizer optimizer(configuration, configuration);

  GlobalMappingRequest request;
  request.request_id = 17;
  request.source_pose_version = 3;
  request.source_map_state_version = 8;
  request.session_id = 2;
  request.maximum_iteration_count = 1;
  request.worker_thread_count = 1;
  auto first_cloud = std::make_shared<sensor::PointCloudXYZI>();
  auto second_cloud = std::make_shared<sensor::PointCloudXYZI>();
  request.initial_poses.resize(2);
  request.keyframe_indices = {0, 1};
  request.keyframes = {{0, 2, first_cloud}, {1, 2, second_cloud}};

  const GlobalMappingResult result = optimizer.OptimizeLocalWindow(request, {2});
  EXPECT_EQ(result.status, GlobalMappingStatus::kSucceeded);
  EXPECT_TRUE(result.MatchesRequest(request));
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
