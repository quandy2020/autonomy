/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping_state.hpp"

#include <algorithm>
#include <utility>

namespace atlas {
namespace backend {
namespace global_mapping {

std::size_t GlobalMappingState::next_keyframe_index() const {
  return next_keyframe_index_;
}

const PgoEdges& GlobalMappingState::first_edges() const { return first_edges_; }

const PgoEdges& GlobalMappingState::second_edges() const {
  return second_edges_;
}

const std::vector<std::unique_ptr<map::Keyframe>>& GlobalMappingState::submaps()
    const {
  return submaps_;
}

bool GlobalMappingState::CommitLocalWindow(
    const GlobalMappingRequest& request, GlobalMappingResult* result,
    std::uint64_t* pose_version, std::uint64_t* map_version,
    int current_session_id,
    const std::vector<std::unique_ptr<map::Keyframe>>& keyframes,
    std::unique_ptr<map::Keyframe> submap, std::size_t consumed_count) {
  if (result == nullptr || pose_version == nullptr || map_version == nullptr ||
      submap == nullptr || consumed_count == 0 ||
      next_keyframe_index_ > keyframes.size()) {
    return false;
  }
  submaps_.reserve(submaps_.size() + 1);
  if (!committer_.Commit(request, *result, *pose_version, *map_version,
                         current_session_id, keyframes)) {
    return false;
  }
  first_edges_ = std::move(result->optimized_edges);
  submaps_.push_back(std::move(submap));
  const std::size_t remaining_count = keyframes.size() - next_keyframe_index_;
  next_keyframe_index_ += std::min(consumed_count, remaining_count);
  ++*pose_version;
  ++*map_version;
  return true;
}

bool GlobalMappingState::CommitSubmapOptimization(
    const GlobalMappingRequest& request, GlobalMappingResult* result,
    std::uint64_t* pose_version, std::uint64_t* map_version,
    int current_session_id) {
  if (result == nullptr || pose_version == nullptr || map_version == nullptr ||
      !committer_.Commit(request, *result, *pose_version, *map_version,
                         current_session_id, submaps_)) {
    return false;
  }
  second_edges_ = std::move(result->optimized_edges);
  ++*pose_version;
  ++*map_version;
  return true;
}

bool GlobalMappingState::ClearSecondEdges(const GlobalMappingRequest& request,
                                          std::uint64_t current_pose_version,
                                          std::uint64_t* current_map_version,
                                          int current_session_id) {
  if (current_map_version == nullptr ||
      request.source_pose_version != current_pose_version ||
      request.source_map_state_version != *current_map_version ||
      request.session_id != current_session_id) {
    return false;
  }
  if (!second_edges_.edges.empty()) {
    second_edges_ = PgoEdges{};
    ++*current_map_version;
  }
  return true;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
