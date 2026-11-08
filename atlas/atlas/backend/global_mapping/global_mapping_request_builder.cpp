/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping_request_builder.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace atlas {
namespace backend {
namespace global_mapping {

void GlobalMappingRequestBuilder::AddKeyframe(const map::Keyframe& keyframe,
                                              std::size_t keyframe_index,
                                              GlobalMappingRequest* request) {
  request->initial_poses.push_back(keyframe.inertial_state);
  request->keyframe_indices.push_back(keyframe_index);
  BundleAdjustmentKeyframe keyframe_snapshot;
  keyframe_snapshot.keyframe_id = keyframe.keyframe_id;
  keyframe_snapshot.session_id = keyframe.session_id;
  keyframe_snapshot.cloud = keyframe.point_cloud;
  request->keyframes.push_back(std::move(keyframe_snapshot));
}

GlobalMappingRequest GlobalMappingRequestBuilder::BuildLocalWindowRequest(
    GlobalMappingRequest request, std::size_t first_keyframe_index,
    std::size_t maximum_keyframe_count,
    const std::vector<std::unique_ptr<map::Keyframe>>& keyframes) const {
  if (first_keyframe_index > keyframes.size()) {
    throw std::logic_error(
        "global mapping keyframe cursor exceeds trajectory size");
  }
  request.initial_poses.clear();
  request.keyframe_indices.clear();
  request.keyframes.clear();
  const std::size_t pending_count = keyframes.size() - first_keyframe_index;
  const std::size_t window_count =
      std::min(maximum_keyframe_count, pending_count);
  request.initial_poses.reserve(window_count);
  request.keyframe_indices.reserve(window_count);
  request.keyframes.reserve(window_count);
  for (std::size_t keyframe_index = first_keyframe_index;
       keyframe_index < first_keyframe_index + window_count;
       ++keyframe_index) {
    if (keyframes[keyframe_index] != nullptr) {
      AddKeyframe(*keyframes[keyframe_index], keyframe_index, &request);
    }
  }
  return request;
}

GlobalMappingRequest GlobalMappingRequestBuilder::BuildSubmapRequest(
    GlobalMappingRequest request,
    const std::vector<std::unique_ptr<map::Keyframe>>& submaps) const {
  request.initial_poses.clear();
  request.keyframe_indices.clear();
  request.keyframes.clear();
  request.initial_poses.reserve(submaps.size());
  request.keyframe_indices.reserve(submaps.size());
  request.keyframes.reserve(submaps.size());
  for (std::size_t submap_index = 0; submap_index < submaps.size();
       ++submap_index) {
    if (submaps[submap_index] != nullptr) {
      AddKeyframe(*submaps[submap_index], submap_index, &request);
    }
  }
  return request;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
