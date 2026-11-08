/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_RESULT_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_RESULT_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "backend/global_mapping/global_mapping_request.hpp"
#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/inertial/inertial_state.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

using ::atlas::frontend::inertial::InertialState;

enum class GlobalMappingStatus {
  kSucceeded,
  kStale,
  kFailed,
};

/** @brief Value result produced from one versioned global mapping request. */
struct GlobalMappingResult {
  std::uint64_t request_id = 0;
  std::uint64_t source_pose_version = 0;
  std::uint64_t source_map_state_version = 0;
  /** @brief Session identity captured by the source request. */
  int source_session_id = -1;
  GlobalMappingStatus status = GlobalMappingStatus::kFailed;
  std::vector<size_t> keyframe_indices;
  std::vector<InertialState> optimized_poses;
  PgoEdges optimized_edges;
  std::string diagnostic;

  /** @brief Check identity, cardinality, finite states, and edge structure. */
  bool MatchesRequest(const GlobalMappingRequest& request) const {
    if (status != GlobalMappingStatus::kSucceeded ||
        request_id != request.request_id ||
        source_pose_version != request.source_pose_version ||
        source_map_state_version != request.source_map_state_version ||
        source_session_id != request.session_id ||
        keyframe_indices != request.keyframe_indices ||
        request.keyframes.size() != request.keyframe_indices.size() ||
        optimized_poses.size() != request.initial_poses.size() ||
        optimized_poses.size() != request.keyframes.size()) {
      return false;
    }
    for (const InertialState& state : optimized_poses) {
      if (!std::isfinite(state.t) || !state.R.allFinite() ||
          !state.p.allFinite() || !state.v.allFinite() ||
          !state.bg.allFinite() || !state.ba.allFinite() ||
          !state.g.allFinite() || !state.cov.allFinite()) {
        return false;
      }
    }
    for (const PgoEdge& edge : optimized_edges.edges) {
      const std::size_t constraint_count = edge.rots.size();
      if (edge.m1 < 0 || edge.m2 < 0 || edge.ids1.size() != constraint_count ||
          edge.ids2.size() != constraint_count ||
          edge.tras.size() != constraint_count ||
          edge.covs.size() != constraint_count) {
        return false;
      }
      for (std::size_t constraint_index = 0;
           constraint_index < constraint_count; ++constraint_index) {
        if (edge.ids1[constraint_index] < 0 ||
            edge.ids2[constraint_index] < 0 ||
            !edge.rots[constraint_index].allFinite() ||
            !edge.tras[constraint_index].allFinite() ||
            !edge.covs[constraint_index].allFinite() ||
            edge.covs[constraint_index].minCoeff() < 0.0) {
          return false;
        }
      }
    }
    return true;
  }

  /** @brief Check that a valid result still targets the active map state. */
  bool MatchesCurrentState(const GlobalMappingRequest& request,
                           std::uint64_t current_pose_version,
                           std::uint64_t current_map_state_version,
                           int current_session_id) const {
    return MatchesRequest(request) &&
           source_pose_version == current_pose_version &&
           source_map_state_version == current_map_state_version &&
           source_session_id == current_session_id;
  }
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_RESULT_HPP_
