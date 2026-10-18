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
 * @file voxel_octo_tree.hpp
 * @brief Adaptive octree node holding points / a fitted plane.
 */

#ifndef ARGUS_MAP_VOXEL_OCTO_TREE_HPP_
#define ARGUS_MAP_VOXEL_OCTO_TREE_HPP_

#include <vector>

#include <Eigen/Dense>

#include "argus/frontend/types.hpp"
#include "argus/map/map_point_confidence.hpp"
#include "argus/map/voxel_plane.hpp"

namespace argus {
namespace map {

using frontend::PointWithVar;

/** @brief Octree node for incremental plane fitting in a voxel. */
class VoxelOctoTree
{
public:
    VoxelOctoTree() = default;
    std::vector<PointWithVar> temp_points_;
    VoxelPlane* plane_ptr_ = nullptr;
    int layer_ = 0;
    int octo_state_ = 0;
    VoxelOctoTree* leaves_[8] = {nullptr, nullptr, nullptr, nullptr,
                                 nullptr, nullptr, nullptr, nullptr};
    double voxel_center_[3] = {0, 0, 0};
    std::vector<int> layer_init_num_;
    float quater_length_ = 0;
    float planer_threshold_ = 0;
    int points_size_threshold_ = 0;
    int update_size_threshold_ = 5;
    int max_points_num_ = 0;
    int max_layer_ = 0;
    int new_points_ = 0;
    bool init_octo_ = false;
    bool update_enable_ = true;
    /** @brief Root-voxel confidence for self-cleaning (P5). */
    MapPointConfidence confidence_;

    VoxelOctoTree(int max_layer, int layer, int points_size_threshold,
                  int max_points_num, float planer_threshold);
    ~VoxelOctoTree();
    VoxelOctoTree(const VoxelOctoTree&) = delete;
    VoxelOctoTree& operator=(const VoxelOctoTree&) = delete;

    void init_plane(const std::vector<PointWithVar>& points, VoxelPlane* plane);
    void init_octo_tree();
    void cut_octo_tree();
    void UpdateOctoTree(const PointWithVar& pv);
    VoxelOctoTree* find_correspond(Eigen::Vector3d pw);
    VoxelOctoTree* Insert(const PointWithVar& pv);
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_OCTO_TREE_HPP_
