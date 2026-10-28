/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_LOOP_UPDATE_HPP_
#define ARGUS_VXSLAM_LOOP_UPDATE_HPP_

#include <unordered_map>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "map/voxel/sliding_octree.hpp"
#include "map/voxel/voxel_key_hash.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Rebuild sliding map after loop pose update (Voxel-SLAM loop_update
 * core).
 *
 * Clears feat_map, re-cuts buffered scans with updated poses, Recut leaves.
 */
void LoopUpdateMap(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                   const std::vector<ImuState>& x_buf,
                   const std::vector<PointVarListPtr>& pvec_buf, int win_count,
                   int win_size, std::vector<FrameWindow*>* sw_pool);

}  // namespace vxslam
}  // namespace argus

#endif
