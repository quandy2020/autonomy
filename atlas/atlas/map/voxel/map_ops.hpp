/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_MAP_OPS_HPP_
#define ATLAS_VXSLAM_MAP_OPS_HPP_

#include <unordered_map>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "map/voxel/sliding_octree.hpp"
#include "map/voxel/voxel_key_hash.hpp"

namespace atlas {
namespace vxslam {

/** @brief Multi-thread marginalize oldest scan (Voxel-SLAM multi_margi). */
void MultiMarginalize(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                      double jour, int win_count, std::vector<ImuState>* xs,
                      LidarFactor* voxopt, std::vector<FrameWindow*>* sw,
                      int thread_num);

/** @brief Multi-thread Recut + CollectFactors (Voxel-SLAM multi_recut). */
void MultiRecut(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                int win_count, std::vector<ImuState>* xs, LidarFactor* voxopt,
                std::vector<std::vector<FrameWindow*>>* sws, int thread_num);

}  // namespace vxslam
}  // namespace atlas

#endif
