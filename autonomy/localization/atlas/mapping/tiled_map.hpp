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

//! mapping/tiled_map — chunked world map with PCD + index.yaml persistence
//! (lightning-lm TiledMap / MapChunk style).

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

//! Static / dynamic point layers within a tile (optional dyn PCD).
enum class TileLayer { kStatic, kDynamic, kAll };

//! Dynamic-layer retention (Lightning-style). Default permanent = keep forever.
enum class DynPolicy { kShort = 0, kMedium = 1, kPermanent = 2 };

inline DynPolicy ParseDynPolicy(const std::string& s) {
    if (s == "short" || s == "Short" || s == "SHORT") {
        return DynPolicy::kShort;
    }
    if (s == "medium" || s == "Medium" || s == "MEDIUM") {
        return DynPolicy::kMedium;
    }
    return DynPolicy::kPermanent;
}

inline const char* DynPolicyName(DynPolicy p) {
    switch (p) {
        case DynPolicy::kShort:
            return "short";
        case DynPolicy::kMedium:
            return "medium";
        default:
            return "permanent";
    }
}

class TiledMap {
public:
    using Key = std::int64_t;

    struct Options {
        double tile_size_m = 50.0;
        //! Leaf size for voxel downsample on insert / save.
        double voxel_size_m = 0.1;
        std::size_t max_points_per_tile = 200000;
        bool use_ivox_per_tile = false;
        //! Coarse 2D occupancy hit counts within each tile (optional).
        bool store_occupancy = false;
        double occupancy_cell_m = 0.5;
        //! PCD writer: binary compressed vs ascii.
        bool save_binary_compressed = true;
        //! LoadOnPose: drop tiles farther than radius (+ margin).
        bool unload_far_tiles = true;
        double unload_margin_m = 0.0;
        //! Split body.z() > dyn_body_z_thresh into dyn_points (default off).
        bool enable_dyn_layer = false;
        double dyn_body_z_thresh = 1.5;
        //! Retention for dyn_points (Lightning short/medium/permanent).
        DynPolicy dyn_policy = DynPolicy::kPermanent;
        double dyn_short_ttl_s = 2.0;
        double dyn_medium_ttl_s = 30.0;
        IVox::Options ivox;
    };

    //! On-disk catalog entry (lazy LoadOnPose) and in-memory tile payload.
    struct Tile {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Key key = 0;
        int ix = 0;
        int iy = 0;
        //! Static (or legacy single-layer) points.
        std::vector<Vec3_t> points;
        //! Dynamic layer (only when Options::enable_dyn_layer).
        std::vector<Vec3_t> dyn_points;
        //! Wall/scan time of last dyn insert (for short/medium TTL).
        double dyn_last_insert_t = 0.0;
        Vec3_t min_bound = Vec3_t::Constant(1e30);
        Vec3_t max_bound = Vec3_t::Constant(-1e30);
        //! Relative PCD path under map dir (e.g. "chunk_0_0.pcd").
        std::string filename;
        //! Optional dyn PCD (e.g. "chunk_0_0_dyn.pcd"); empty = none.
        std::string dyn_filename;
        bool loaded = false;
        //! Optional occupancy: packed local (cx,cy) → hit count.
        std::unordered_map<Key, float> occupancy;
        //! Owned separately so Tile stays move-insertable into unordered_map.
        std::unique_ptr<IVox> ivox;

        explicit Tile(IVox::Options opts = {})
            : ivox(std::make_unique<IVox>(std::move(opts))) {}

        void ExpandBound(const Vec3_t& p) {
            min_bound = min_bound.cwiseMin(p);
            max_bound = max_bound.cwiseMax(p);
        }

        [[nodiscard]] Vec3_t Center() const {
            if (points.empty() && dyn_points.empty() &&
                min_bound.x() > max_bound.x()) {
                return Vec3_t::Zero();
            }
            return 0.5 * (min_bound + max_bound);
        }
    };

    TiledMap() = default;
    explicit TiledMap(Options options);

    //! Chunk key helpers (XY tiles; Z reserved for future 3D tiling).
    [[nodiscard]] Key ChunkKeyFromXYZ(double x, double y, double z = 0.0) const;
    [[nodiscard]] Key ChunkKeyFromPose(const Mat44_t& T_wb) const;
    [[nodiscard]] static Key PackKey(int ix, int iy);
    void IndexOf(const Vec3_t& p, int* ix, int* iy) const;

    //! Transform body→world and route points into tiles (voxel-downsampled).
    //! When enable_dyn_layer, body.z() > dyn_body_z_thresh → dyn_points.
    //! @param t_sec  Scan time for dyn TTL (≤0 → wall clock).
    int IntegrateScan(const Mat44_t& T_wb,
                      const std::vector<Vec3_t>& points_body,
                      double t_sec = -1.0);
    int InsertWorldPoints(const std::vector<Vec3_t>& points_world);
    int InsertWorldPoints(const std::vector<Vec3_t>& points_world,
                          TileLayer layer);

    //! Drop expired dyn points (short/medium policy). No-op if permanent.
    int PruneDyn(double now_sec);

    //! Incremental dyn-layer update from world-frame scan (prior-map Loc).
    //! Enables dyn layer if needed; stamps dyn_last_insert_t; then PruneDyn.
    //! @param remove_old  Unused reserved (Lightning API parity); prune via policy.
    int UpdateDynamicCloud(const std::vector<Vec3_t>& points_world,
                           double t_sec = -1.0, bool remove_old = true);

    //! Write per-chunk PCD (+ optional _dyn.pcd) + index.yaml under `dir`.
    bool Save(const std::string& dir) const;
    //! Restore catalog + chunks from index.yaml + PCD files.
    bool Load(const std::string& dir);

    //! Lazy-load tiles whose centers lie within `radius` of `pos`; optionally
    //! unload tiles farther than radius + unload_margin_m.
    void LoadOnPose(const Vec3_t& pos, double radius);

    //! Collect currently loaded points for a layer (for LidarLocator NDT).
    void CollectLoadedPoints(TileLayer layer,
                             std::vector<Vec3_t>* out) const;

    void Clear();
    [[nodiscard]] std::size_t num_tiles() const;
    [[nodiscard]] std::size_t num_loaded_tiles() const;
    [[nodiscard]] std::size_t num_points() const;

    Options& options() { return options_; }
    [[nodiscard]] const Options& options() const { return options_; }
    [[nodiscard]] const std::string& map_dir() const { return map_dir_; }

    //! Non-owning view of currently loaded tiles (caller must not unlock).
    std::unordered_map<Key, Tile>& tiles() { return tiles_; }
    [[nodiscard]] const std::unordered_map<Key, Tile>& tiles() const {
        return tiles_;
    }

private:
    Tile& EnsureTileLocked(int ix, int iy);
    void AddPointLocked(Tile& tile, const Vec3_t& p, TileLayer layer);
    void DownsampleTileLocked(Tile& tile);
    bool LoadTilePcdLocked(Tile& tile);
    void UnloadTileLocked(Tile& tile);
    [[nodiscard]] Vec3_t TileCenterEstimate(int ix, int iy) const;
    Key OccupancyKey(int ix, int iy, const Vec3_t& p) const;

    Options options_;
    mutable std::mutex mtx_;
    std::string map_dir_;
    //! All known tiles (metadata + optional points).
    std::unordered_map<Key, Tile> tiles_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
