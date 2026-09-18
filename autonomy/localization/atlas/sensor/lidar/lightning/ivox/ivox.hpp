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
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {
namespace sensor {
namespace lightning {

/**
 * Lightweight incremental voxel hash (IVox-shaped) for lidar map points.
 * Full Faster-LIO IVox lands under upstream/; this grid is enough for
 * point-plane correspondence inside the single AtlasSystem.
 */
class IVox {
public:
    struct Options {
        double resolution = 0.5;
        std::size_t max_points_per_voxel = 20;
        std::size_t max_voxels = 200000;
        int neighbor_search = 1;  // Chebyshev radius in voxel indices
        int min_plane_points = 5;
    };

    struct PlaneHit {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool ok = false;
        Vec3_t normal = Vec3_t::UnitZ();
        double d = 0.0;
        double residual = 0.0;
    };

    IVox();
    explicit IVox(Options options);

    void Clear();
    void InsertWorldPoints(const std::vector<Vec3_t>& points_world);
    [[nodiscard]] std::size_t num_voxels() const;
    [[nodiscard]] std::size_t num_points() const;

    //! Estimate local plane around a world query point (PCA of neighbor voxels).
    [[nodiscard]] PlaneHit EstimatePlane(const Vec3_t& query_world) const;

private:
    using Key = std::int64_t;
    struct Voxel {
        std::vector<Vec3_t> points;
    };

    Key ToKey(int ix, int iy, int iz) const;
    void IndexOf(const Vec3_t& p, int* ix, int* iy, int* iz) const;
    void CollectNeighbors(int ix, int iy, int iz,
                          std::vector<Vec3_t>* out) const;

    Options options_;
    mutable std::mutex mtx_;
    std::unordered_map<Key, Voxel> voxels_;
    std::size_t num_points_ = 0;
};

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas
