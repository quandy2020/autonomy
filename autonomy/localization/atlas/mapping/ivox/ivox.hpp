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

//! Incremental sparse voxels (IVox) for lidar local map — lightning-lm /
//! Faster-LIO shaped. Outer hash + LRU; per-voxel linear or PHC node;
//! Nearby6/18/26 neighbor gather; Morton/Hilbert optional key packing.

#include "autonomy/localization/atlas/mapping/ivox/ivox_node.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

enum class IVoxNearbyType {
    kCenter = 0,
    kNearby6 = 1,
    kNearby18 = 2,
    kNearby26 = 3,
};

enum class IVoxNodeKind {
    kLinear = 0,
    kPhc = 1,
};

inline IVoxNearbyType ParseIVoxNearbyType(const std::string& s) {
    if (s == "center" || s == "CENTER" || s == "0") {
        return IVoxNearbyType::kCenter;
    }
    if (s == "nearby18" || s == "NEARBY18" || s == "18") {
        return IVoxNearbyType::kNearby18;
    }
    if (s == "nearby26" || s == "NEARBY26" || s == "26") {
        return IVoxNearbyType::kNearby26;
    }
    return IVoxNearbyType::kNearby6;
}

inline IVoxNodeKind ParseIVoxNodeKind(const std::string& s) {
    if (s == "phc" || s == "PHC" || s == "hilbert") {
        return IVoxNodeKind::kPhc;
    }
    return IVoxNodeKind::kLinear;
}

inline const char* IVoxNearbyTypeName(IVoxNearbyType t) {
    switch (t) {
        case IVoxNearbyType::kCenter:
            return "center";
        case IVoxNearbyType::kNearby18:
            return "nearby18";
        case IVoxNearbyType::kNearby26:
            return "nearby26";
        default:
            return "nearby6";
    }
}

inline const char* IVoxNodeKindName(IVoxNodeKind k) {
    return k == IVoxNodeKind::kPhc ? "phc" : "linear";
}

class IVox {
public:
    struct Options {
        double resolution = 0.5;
        std::size_t max_points_per_voxel = 20;
        std::size_t max_voxels = 200000;
        //! Legacy Chebyshev radius; ignored when nearby_type is set (preferred).
        int neighbor_search = 1;
        int min_plane_points = 5;
        //! Pack voxel indices with Morton (Z-order) instead of linear pack.
        bool use_morton_key = false;
        //! Pack with Hilbert curve index (takes precedence over Morton).
        bool use_hilbert_key = false;
        //! Neighbor stencil (lightning NearbyType). Default NEARBY6.
        IVoxNearbyType nearby_type = IVoxNearbyType::kNearby6;
        //! Per-voxel storage: linear ring or PHC sub-cubes.
        IVoxNodeKind node_kind = IVoxNodeKind::kLinear;
        //! PHC subdivision order (1..8); only used when node_kind=phc.
        int phc_order = 6;
        //! KNN radius (m) for GetClosestPoints / EstimatePlane.
        double knn_max_range = 5.0;
        //! Max KNN neighbors for plane fit (capped by min_plane_points when larger).
        int knn_max_num = 5;
        //! Reject plane if any neighbor is farther than this from the plane
        //! (lightning ESTI_PLANE_THRESHOLD). 0 disables the gate.
        double esti_plane_threshold = 0.1;
        //! Deprecated alias: face+chebyshev; prefer nearby_type.
        bool face_adjacent_neighbors = true;
    };

    struct PlaneHit {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool ok = false;
        Vec3_t normal = Vec3_t::UnitZ();
        double d = 0.0;
        double residual = 0.0;
    };

    struct CapacityStats {
        std::size_t last_neighbor_points = 0;
        std::size_t last_neighbor_voxels = 0;
        std::size_t peak_neighbor_points = 0;
        std::size_t num_evictions = 0;
    };

    IVox();
    explicit IVox(Options options);

    void set_options(Options options);
    [[nodiscard]] const Options& options() const { return options_; }

    void Clear();
    void InsertWorldPoints(const std::vector<Vec3_t>& points_world);

    //! Export all stored world points (optional stride downsample to max_points).
    void ExportWorldPoints(std::vector<Vec3_t>* out,
                           std::size_t max_points = 0) const;

    //! Approximate KNN across nearby voxels (lightning GetClosestPoint).
    bool GetClosestPoints(const Vec3_t& query, std::vector<Vec3_t>* closest,
                          int max_num = 5, double max_range = 5.0) const;

    [[nodiscard]] std::size_t num_voxels() const;
    [[nodiscard]] std::size_t num_points() const;
    [[nodiscard]] CapacityStats capacity_stats() const;

    //! Estimate local plane around a world query (PCA of KNN neighbors).
    [[nodiscard]] PlaneHit EstimatePlane(const Vec3_t& query_world) const;

private:
    struct Key3 {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const Key3& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct Key3Hash {
        std::size_t operator()(const Key3& k) const noexcept;
    };

    using Node = std::variant<IVoxNodeLinear, IVoxNodePhc>;
    using CacheEntry = std::pair<Key3, Node>;
    using CacheList = std::list<CacheEntry>;
    using CacheIter = CacheList::iterator;

    Key3 Pos2Grid(const Vec3_t& p) const;
    void GenerateNearbyGrids();
    Node MakeNode(const Key3& key) const;
    void InsertOne(const Vec3_t& p);
    void TouchLru(CacheIter it);

    Options options_;
    double inv_resolution_ = 2.0;
    mutable std::mutex mtx_;
    CacheList cache_;
    std::unordered_map<Key3, CacheIter, Key3Hash> grids_;
    std::vector<Key3> nearby_grids_;
    std::size_t num_points_ = 0;
    mutable CapacityStats stats_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
