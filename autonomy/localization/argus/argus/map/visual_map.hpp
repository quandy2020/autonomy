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
 * @file visual_map.hpp
 * @brief Sparse visual voxel map (FAST-LIVO2 feat_map).
 *
 * Related types: `voxel_points.hpp`, `sub_sparse_map.hpp`,
 * `visual_point.hpp`, `visual_feature.hpp`.
 */

#ifndef ARGUS_MAP_VISUAL_MAP_HPP_
#define ARGUS_MAP_VISUAL_MAP_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/map/sub_sparse_map.hpp"
#include "argus/map/visual_point.hpp"
#include "argus/map/voxel_location.hpp"
#include "argus/map/voxel_points.hpp"

namespace argus {
namespace map {

/**
 * @brief Sparse visual map hashed by `VoxelLocation` (0.5 m cells by default).
 */
class VisualMap
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(VisualMap);

    /**
     * @brief Construct map with given voxel edge length.
     * @param voxel_size Voxel size in meters (default 0.5).
     */
    explicit VisualMap(double voxel_size = 0.5);
    ~VisualMap();

    VisualMap(const VisualMap&) = delete;
    VisualMap& operator=(const VisualMap&) = delete;

    /** @param voxel_size Voxel edge length (m). */
    void SetVoxelSize(double voxel_size) {
        voxel_size_ = voxel_size;
    }
    /** @return Current voxel edge length (m). */
    double voxel_size() const {
        return voxel_size_;
    }

    /**
     * @brief Insert a visual point; takes ownership via VoxelPoints buckets.
     * @param pt_new Heap-allocated point (map owns via voxel bucket).
     */
    void InsertPoint(VisualPoint* pt_new);

    /** @brief Remove all voxels and destroy contained landmarks. */
    void Clear();

    /** @return Number of occupied voxels (not landmark count). */
    std::size_t size() const {
        return feat_map_.size();
    }

    /** @return Total number of VisualPoint landmarks. */
    std::size_t LandmarkCount() const;

    /** @return True if no voxels are stored. */
    bool empty() const {
        return feat_map_.empty();
    }

    /** @return Mutable voxel hash map (VoxelLocation → VoxelPoints*). */
    std::unordered_map<VoxelLocation, VoxelPoints*>& feat_map() {
        return feat_map_;
    }
    /** @return Const voxel hash map. */
    const std::unordered_map<VoxelLocation, VoxelPoints*>& feat_map() const {
        return feat_map_;
    }

    /**
     * @brief Voxel key for a world-frame 3D point.
     * @param pt_w Point in world coordinates (m).
     * @return Discretized voxel location.
     */
    VoxelLocation KeyForPoint(const Eigen::Vector3d& pt_w) const;

    /**
     * @brief Export landmarks as `x y z nx ny nz` (no patches).
     * @param path Output file path.
     * @return True on successful write.
     * @note Patch / Feature blobs are intentionally omitted for map IO.
     */
    bool SaveLandmarks(const std::string& path) const;

    /**
     * @brief Load landmarks; clears existing map and inserts points without
     * obs.
     * @param path Input file path.
     * @return True on successful read.
     */
    bool LoadLandmarks(const std::string& path);

private:
    double voxel_size_ = 0.5;
    std::unordered_map<VoxelLocation, VoxelPoints*> feat_map_;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VISUAL_MAP_HPP_
