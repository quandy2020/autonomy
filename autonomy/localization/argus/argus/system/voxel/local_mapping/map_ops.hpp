/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_MAP_OPS_HPP_
#define ARGUS_VXSLAM_MAP_OPS_HPP_

#include <unordered_map>
#include <vector>

#include "argus/system/voxel/odometry/imu_state.hpp"
#include "argus/system/voxel/local_mapping/lidar_factor.hpp"
#include "argus/system/voxel/local_mapping/sliding_octree.hpp"
#include "argus/mapping/voxel_key.hpp"

namespace argus {
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
}  // namespace argus

#endif
