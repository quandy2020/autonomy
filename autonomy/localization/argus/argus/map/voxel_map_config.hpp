/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file voxel_map_config.hpp
 * @brief Configuration for adaptive voxel / plane LIO map.
 */

#ifndef ARGUS_MAP_VOXEL_MAP_CONFIG_HPP_
#define ARGUS_MAP_VOXEL_MAP_CONFIG_HPP_

#include <vector>

namespace argus {
namespace map {

/** @brief Voxel resolution, plane fit, and residual thresholds for LIO. */
struct VoxelMapConfig {
    double max_voxel_size_ = 0.5; /**< @brief Root voxel edge length (m). */
    int max_layer_ = 1;           /**< @brief Max octree depth. */
    int max_iterations_ = 5;      /**< @brief ESIKF / residual iterations. */
    std::vector<int> layer_init_num_{5, 5, 5, 5,
                                     5}; /**< @brief Min pts per layer. */
    int max_points_num_ = 50;
    double planner_threshold_ = 0.01;
    double beam_err_ = 0.02;
    double dept_err_ = 0.05;
    double sigma_num_ = 3;
    bool is_pub_plane_map_ = false;
    double sliding_thresh_ = 8;
    bool map_sliding_en_ = false;
    int half_map_size_ = 100;
};

/** @brief Fill defaults matching historical FAST-LIVO2 YAML. */
void SetDefaultVoxelMapConfig(VoxelMapConfig* config);

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_MAP_CONFIG_HPP_
