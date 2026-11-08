/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_STATE_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_STATE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "backend/global_mapping/global_mapping_committer.hpp"
#include "backend/global_mapping/global_mapping_request.hpp"
#include "backend/global_mapping/global_mapping_result.hpp"
#include "backend/global_mapping/pgo_edges.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

/**
 * @brief Owns HBA cursor, submaps, and accumulated global mapping edges.
 * @note The caller must hold the mutex protecting the keyframes and versions.
 */
class GlobalMappingState {
 public:
  std::size_t next_keyframe_index() const;
  const PgoEdges& first_edges() const;
  const PgoEdges& second_edges() const;
  const std::vector<std::unique_ptr<map::Keyframe>>& submaps() const;

  bool CommitLocalWindow(
      const GlobalMappingRequest& request, GlobalMappingResult* result,
      std::uint64_t* pose_version, std::uint64_t* map_version,
      int current_session_id,
      const std::vector<std::unique_ptr<map::Keyframe>>& keyframes,
      std::unique_ptr<map::Keyframe> submap, std::size_t consumed_count);
  bool CommitSubmapOptimization(const GlobalMappingRequest& request,
                                GlobalMappingResult* result,
                                std::uint64_t* pose_version,
                                std::uint64_t* map_version,
                                int current_session_id);
  bool ClearSecondEdges(const GlobalMappingRequest& request,
                        std::uint64_t current_pose_version,
                        std::uint64_t* current_map_version,
                        int current_session_id);

 private:
  std::size_t next_keyframe_index_ = 0;
  PgoEdges first_edges_;
  PgoEdges second_edges_;
  std::vector<std::unique_ptr<map::Keyframe>> submaps_;
  GlobalMappingCommitter committer_;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_STATE_HPP_
