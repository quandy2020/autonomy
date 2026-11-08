/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping_committer.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

bool GlobalMappingCommitter::Commit(
    const GlobalMappingRequest& request, const GlobalMappingResult& result,
    std::uint64_t current_pose_version, std::uint64_t current_map_state_version,
    int current_session_id,
    const std::vector<std::unique_ptr<map::Keyframe>>& keyframes) const {
  if (!result.MatchesCurrentState(request, current_pose_version,
                                  current_map_state_version,
                                  current_session_id)) {
    return false;
  }
  for (std::size_t request_index = 0;
       request_index < request.keyframe_indices.size(); ++request_index) {
    const std::size_t keyframe_index = request.keyframe_indices[request_index];
    if (keyframe_index >= keyframes.size() ||
        keyframes[keyframe_index] == nullptr ||
        keyframes[keyframe_index]->keyframe_id !=
            request.keyframes[request_index].keyframe_id ||
        keyframes[keyframe_index]->session_id !=
            request.keyframes[request_index].session_id) {
      return false;
    }
  }
  for (std::size_t request_index = 0;
       request_index < request.keyframe_indices.size(); ++request_index) {
    keyframes[request.keyframe_indices[request_index]]->inertial_state =
        result.optimized_poses[request_index];
  }
  return true;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
