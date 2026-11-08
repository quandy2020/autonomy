/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_KEYFRAME_LOAD_HPP_
#define ATLAS_MAP_KEYFRAME_LOAD_HPP_

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include "map/sliding_voxel_tree.hpp"
#include "map/voxel_key_hash.hpp"

namespace atlas {
namespace map {

/**
 * @brief Reload one nearby history keyframe into the sliding map
 *        (Voxel-SLAM keyframe_loading).
 *
 * @return true if a keyframe was injected.
 */
bool KeyframeLoading(SlidingVoxelMap* feat_map,
                     const std::vector<Keyframe*>& keyframes,
                     const Eigen::Vector3d& curr_p, double radius_m,
                     int win_size, int* history_kfsize, double jour,
                     std::shared_ptr<const SlidingVoxelMapOptions> map_options);

}  // namespace map
}  // namespace atlas

#endif
