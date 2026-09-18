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
        IVox::Options ivox;
    };

    //! On-disk catalog entry (lazy LoadOnPose) and in-memory tile payload.
    struct Tile {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Key key = 0;
        int ix = 0;
        int iy = 0;
        std::vector<Vec3_t> points;
        Vec3_t min_bound = Vec3_t::Constant(1e30);
        Vec3_t max_bound = Vec3_t::Constant(-1e30);
        //! Relative PCD path under map dir (e.g. "chunk_0_0.pcd").
        std::string filename;
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
            if (points.empty() && min_bound.x() > max_bound.x()) {
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
    int IntegrateScan(const Mat44_t& T_wb,
                      const std::vector<Vec3_t>& points_body);
    int InsertWorldPoints(const std::vector<Vec3_t>& points_world);

    //! Write per-chunk PCD + index.yaml under `dir`.
    bool Save(const std::string& dir) const;
    //! Restore catalog + chunks from index.yaml + PCD files.
    bool Load(const std::string& dir);

    //! Lazy-load tiles whose centers lie within `radius` of `pos`; optionally
    //! unload tiles farther than radius + unload_margin_m.
    void LoadOnPose(const Vec3_t& pos, double radius);

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
    void AddPointLocked(Tile& tile, const Vec3_t& p);
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
