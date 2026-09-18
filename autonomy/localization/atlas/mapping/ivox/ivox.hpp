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

#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

/**
 * Lightweight incremental voxel hash (IVox-shaped) for lidar map points.
 * Path: mapping/ivox/; namespace: mapping.
 * Optional Morton / Hilbert key packing for spatial locality on eviction order.
 */
class IVox {
public:
    struct Options {
        double resolution = 0.5;
        std::size_t max_points_per_voxel = 20;
        std::size_t max_voxels = 200000;
        int neighbor_search = 1;  // Chebyshev radius in voxel indices
        int min_plane_points = 5;
        //! Pack voxel indices with Morton (Z-order) instead of linear pack.
        bool use_morton_key = false;
        //! Pack with Hilbert curve index (takes precedence over Morton).
        bool use_hilbert_key = false;
        //! Always include 6 face-adjacent voxels in neighbor gather.
        bool face_adjacent_neighbors = true;
    };

    struct PlaneHit {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool ok = false;
        Vec3_t normal = Vec3_t::UnitZ();
        double d = 0.0;
        double residual = 0.0;
    };

    //! Lightweight capacity / neighbor stats (updated on EstimatePlane).
    struct CapacityStats {
        std::size_t last_neighbor_points = 0;
        std::size_t last_neighbor_voxels = 0;
        std::size_t peak_neighbor_points = 0;
        std::size_t num_evictions = 0;
    };

    IVox();
    explicit IVox(Options options);

    void Clear();
    void InsertWorldPoints(const std::vector<Vec3_t>& points_world);
    [[nodiscard]] std::size_t num_voxels() const;
    [[nodiscard]] std::size_t num_points() const;
    [[nodiscard]] CapacityStats capacity_stats() const;

    //! Estimate local plane around a world query point (PCA of neighbor voxels).
    [[nodiscard]] PlaneHit EstimatePlane(const Vec3_t& query_world) const;

private:
    using Key = std::int64_t;
    struct Voxel {
        std::vector<Vec3_t> points;
    };

    Key ToKey(int ix, int iy, int iz) const;
    static Key MortonEncode3(int ix, int iy, int iz);
    static Key HilbertEncode3(int ix, int iy, int iz);
    void IndexOf(const Vec3_t& p, int* ix, int* iy, int* iz) const;
    void CollectNeighbors(int ix, int iy, int iz,
                          std::vector<Vec3_t>* out,
                          std::size_t* voxel_hits = nullptr) const;
    void EvictOldestLocked();

    Options options_;
    mutable std::mutex mtx_;
    std::unordered_map<Key, Voxel> voxels_;
    //! Insertion-order keys for capacity eviction (LRU-ish: drop oldest).
    std::deque<Key> insert_order_;
    std::size_t num_points_ = 0;
    mutable CapacityStats stats_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
