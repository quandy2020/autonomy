/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_MAP_OPS_HPP_
#define ATLAS_VXSLAM_MAP_OPS_HPP_

#include <unordered_map>
#include <vector>

#include "common/thread_pool.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "map/voxel/sliding_octree.hpp"
#include "map/voxel/voxel_key_hash.hpp"

namespace atlas {
namespace vxslam {

/** @brief Marginalize the oldest scan using a reusable worker pool. */
void MarginalizeSlidingVoxelMap(
    std::unordered_map<VoxelKey, SlidingOctree*>* feature_map,
    double traveled_distance, int window_count, std::vector<ImuState>* states,
    LidarFactor* factor_collection,
    std::vector<FrameWindow*>* frame_window_pool,
    common::ThreadPoolInterface* thread_pool, int worker_count);

/** @brief Recut the map and collect factors using a reusable worker pool. */
void RecutSlidingVoxelMap(
    std::unordered_map<VoxelKey, SlidingOctree*>* feature_map, int window_count,
    std::vector<ImuState>* states, LidarFactor* factor_collection,
    std::vector<std::vector<FrameWindow*>>* frame_window_pools,
    common::ThreadPoolInterface* thread_pool, int worker_count);

}  // namespace vxslam
}  // namespace atlas

#endif
