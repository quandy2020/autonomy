/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping_optimizer.hpp"

#include <vector>
#include <utility>

#include "common/pose_state.hpp"
#include "common/session_pose_constraint.hpp"
#include "frontend/local_mapping/bundle_adjustment_support.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

GlobalMappingResult MakeInitialResult(const GlobalMappingRequest& request) {
  GlobalMappingResult result;
  result.request_id = request.request_id;
  result.source_pose_version = request.source_pose_version;
  result.source_map_state_version = request.source_map_state_version;
  result.source_session_id = request.session_id;
  result.keyframe_indices = request.keyframe_indices;
  result.optimized_poses = request.initial_poses;
  result.optimized_edges = request.accumulated_edges;
  return result;
}

bool ValidateResult(const GlobalMappingRequest& request,
                    GlobalMappingResult* result) {
  result->status = GlobalMappingStatus::kSucceeded;
  if (result->MatchesRequest(request)) return true;
  result->status = GlobalMappingStatus::kFailed;
  result->diagnostic = "global mapping produced an invalid optimization result";
  return false;
}

}  // namespace

GlobalMappingOptimizer::GlobalMappingOptimizer(
    map::GlobalBundleAdjustmentConfiguration local_window_configuration,
    map::GlobalBundleAdjustmentConfiguration submap_configuration)
    : hierarchical_optimizer_(local_window_configuration),
      local_window_configuration_(std::move(local_window_configuration)),
      submap_configuration_(std::move(submap_configuration)) {}

GlobalMappingResult GlobalMappingOptimizer::OptimizeLocalWindow(
    const GlobalMappingRequest& request, const std::vector<int>& session_ids) {
  return OptimizeWindow(request, session_ids, local_window_configuration_);
}

GlobalMappingResult GlobalMappingOptimizer::OptimizeSubmaps(
    const GlobalMappingRequest& request, const std::vector<int>& session_ids) {
  return OptimizeWindow(request, session_ids, submap_configuration_);
}

GlobalMappingResult GlobalMappingOptimizer::OptimizeWindow(
    const GlobalMappingRequest& request, const std::vector<int>& session_ids,
    const map::GlobalBundleAdjustmentConfiguration& configuration) {
  GlobalMappingResult result = MakeInitialResult(request);
  if (request.keyframes.empty() ||
      request.initial_poses.size() != request.keyframes.size() ||
      request.keyframe_indices.size() != request.keyframes.size()) {
    result.diagnostic = "global mapping window request is incomplete";
    return result;
  }
  common::SessionPoseConstraints pose_constraints;
  std::vector<common::PoseState> optimized_pose_window =
      frontend::local_mapping::MakePoseWindow(result.optimized_poses);
  hierarchical_optimizer_.AddEdgesFromHierarchicalBundleAdjustment(
      &optimized_pose_window, request.keyframes, &pose_constraints, session_ids,
      request.maximum_iteration_count, request.worker_thread_count,
      configuration, &local_window_configuration_);
  frontend::local_mapping::ApplyPoseWindow(optimized_pose_window,
                                           &result.optimized_poses);
  AppendSessionPoseConstraints(pose_constraints, &result.optimized_edges);
  ValidateResult(request, &result);
  return result;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
