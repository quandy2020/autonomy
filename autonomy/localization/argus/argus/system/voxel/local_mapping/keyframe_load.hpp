/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_KEYFRAME_LOAD_HPP_
#define ARGUS_VXSLAM_KEYFRAME_LOAD_HPP_

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "argus/system/voxel/local_mapping/sliding_octree.hpp"
#include "argus/mapping/voxel_key.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Reload one nearby history keyframe into the sliding map
 *        (Voxel-SLAM keyframe_loading).
 *
 * @return true if a keyframe was injected.
 */
bool KeyframeLoading(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                     const std::vector<Keyframe*>& keyframes,
                     const Eigen::Vector3d& curr_p, double radius_m,
                     int win_size, int* history_kfsize, double jour);

}  // namespace vxslam
}  // namespace argus

#endif
