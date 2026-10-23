/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_GTSAM_PGO_HPP_
#define ATLAS_VXSLAM_GTSAM_PGO_HPP_

#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/lidar/odometry/scan_pose.hpp"
#include "map/voxel/keyframe.hpp"

namespace atlas {
namespace vxslam {

/**
 * @brief Voxel-SLAM build_graph + ISAM2 (online loop `isOpt` / topDown).
 *
 * Session scan indices are local (0..n-1); edges use local ids + session map
 * ids, converted via stepsizes like upstream.
 *
 * @param inject_gba If true, also inject gba_edges1/2 (topDownProcess).
 * @return false if GTSAM unavailable or graph empty.
 */
bool OptimizeIsam2(std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
                   std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
                   const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
                   const PgoEdges& gba_edges2, int cur_session_id,
                   bool inject_gba);

/** @brief topDownProcess: OptimizeIsam2 with GBA edges. */
inline bool TopDownIsam2(
    std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
    std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
    const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
    const PgoEdges& gba_edges2, int cur_session_id) {
  return OptimizeIsam2(multimap_scans, multimap_keyframes, lp_edges, gba_edges1,
                       gba_edges2, cur_session_id,
                       /*inject_gba=*/true);
}

}  // namespace vxslam
}  // namespace atlas

#endif
