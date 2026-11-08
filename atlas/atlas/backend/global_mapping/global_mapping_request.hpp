/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "map/hierarchical_bundle_adjustment.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::map::BundleAdjustmentKeyframe;

/** @brief Immutable input snapshot for one global mapping optimization pass. */
struct GlobalMappingRequest {
  std::uint64_t request_id = 0;
  std::uint64_t source_pose_version = 0;
  std::uint64_t source_map_state_version = 0;
  int session_id = -1;
  int maximum_iteration_count = 1;
  int worker_thread_count = 1;
  std::vector<size_t> keyframe_indices;
  std::vector<InertialState> initial_poses;
  std::vector<BundleAdjustmentKeyframe> keyframes;
  PgoEdges accumulated_edges;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_HPP_
