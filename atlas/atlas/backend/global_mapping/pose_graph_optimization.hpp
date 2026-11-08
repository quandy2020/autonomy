/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_POSE_GRAPH_OPTIMIZATION_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_POSE_GRAPH_OPTIMIZATION_HPP_

#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/odometry/scan_pose.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::Keyframe;

/**
 * @brief Optimizes online and multi-session pose graphs.
 *
 * Session scan indices are local (0..n-1); edges use local ids + session map
 * ids, converted via stepsizes like upstream.
 *
 * @param inject_gba If true, also inject gba_edges1/2 (topDownProcess).
 * @return false if the graph is empty, malformed, or cannot be optimized.
 */
bool OptimizePoseGraph(std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
                       std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
                       const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
                       const PgoEdges& gba_edges2, int cur_session_id,
                       bool inject_gba);

/** @brief topDownProcess: OptimizePoseGraph with GBA edges. */
inline bool OptimizeTopDownPoseGraph(
    std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
    std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
    const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
    const PgoEdges& gba_edges2, int cur_session_id) {
  return OptimizePoseGraph(multimap_scans, multimap_keyframes, lp_edges,
                           gba_edges1, gba_edges2, cur_session_id,
                           /*inject_gba=*/true);
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif
