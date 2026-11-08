/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_COMMITTER_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_COMMITTER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "backend/global_mapping/global_mapping_request.hpp"
#include "backend/global_mapping/global_mapping_result.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

/**
 * @brief Validates and applies HBA poses to the active keyframes.
 * @note The caller must hold the mutex protecting versions and keyframes.
 */
class GlobalMappingCommitter {
 public:
  bool Commit(
      const GlobalMappingRequest& request, const GlobalMappingResult& result,
      std::uint64_t current_pose_version,
      std::uint64_t current_map_state_version, int current_session_id,
      const std::vector<std::unique_ptr<map::Keyframe>>& keyframes) const;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_COMMITTER_HPP_
