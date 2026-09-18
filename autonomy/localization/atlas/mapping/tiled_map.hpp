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

//! mapping/tiled_map — chunked world map (tile hash → points / optional IVox).

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

class TiledMap {
public:
    struct Options {
        double tile_size_m = 50.0;
        std::size_t max_points_per_tile = 200000;
        bool use_ivox_per_tile = false;
        IVox::Options ivox;
    };

    struct Tile {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::int64_t key = 0;
        int ix = 0;
        int iy = 0;
        std::vector<Vec3_t> points;
        IVox ivox;
        explicit Tile(IVox::Options opts = {}) : ivox(std::move(opts)) {}
    };

    TiledMap() = default;
    explicit TiledMap(Options options) : options_(std::move(options)) {
        if (options_.tile_size_m < 1.0) {
            options_.tile_size_m = 1.0;
        }
    }

    //! Transform body→world and route into the tile under pose translation.
    int IntegrateScan(const Mat44_t& T_wb,
                      const std::vector<Vec3_t>& points_body) {
        if (points_body.empty()) {
            return 0;
        }
        const Mat33_t R = T_wb.block<3, 3>(0, 0);
        const Vec3_t t = T_wb.block<3, 1>(0, 3);
        std::vector<Vec3_t> world;
        world.reserve(points_body.size());
        for (const auto& p : points_body) {
            if (p.allFinite()) {
                world.push_back(R * p + t);
            }
        }
        return InsertWorldPoints(world);
    }

    int InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
        std::lock_guard<std::mutex> lock(mtx_);
        int n = 0;
        for (const auto& p : points_world) {
            if (!p.allFinite()) {
                continue;
            }
            int ix = 0;
            int iy = 0;
            IndexOf(p, &ix, &iy);
            Tile& tile = EnsureTileLocked(ix, iy);
            if (options_.use_ivox_per_tile) {
                tile.ivox.InsertWorldPoints({p});
            } else {
                if (tile.points.size() >= options_.max_points_per_tile) {
                    continue;
                }
                tile.points.push_back(p);
            }
            ++n;
        }
        return n;
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        tiles_.clear();
    }

    [[nodiscard]] std::size_t num_tiles() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return tiles_.size();
    }

    //! Stub: write tile count + options marker (full cloud dump later).
    bool Save(const std::string& path) const {
        std::lock_guard<std::mutex> lock(mtx_);
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs) {
            return false;
        }
        const std::uint64_t n = tiles_.size();
        ofs.write(reinterpret_cast<const char*>(&n), sizeof(n));
        ofs.write(reinterpret_cast<const char*>(&options_.tile_size_m),
                  sizeof(options_.tile_size_m));
        return static_cast<bool>(ofs);
    }

    //! Stub: accept empty / marker files only (does not restore points yet).
    bool Load(const std::string& path) {
        std::lock_guard<std::mutex> lock(mtx_);
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) {
            return false;
        }
        std::uint64_t n = 0;
        double tile_size = options_.tile_size_m;
        ifs.read(reinterpret_cast<char*>(&n), sizeof(n));
        ifs.read(reinterpret_cast<char*>(&tile_size), sizeof(tile_size));
        if (!ifs) {
            return false;
        }
        options_.tile_size_m = tile_size;
        tiles_.clear();
        (void)n;
        return true;
    }

    Options& options() { return options_; }
    [[nodiscard]] const Options& options() const { return options_; }

private:
    using Key = std::int64_t;

    static Key PackKey(int ix, int iy) {
        return (static_cast<std::int64_t>(ix) << 32) ^
               (static_cast<std::uint32_t>(iy));
    }

    void IndexOf(const Vec3_t& p, int* ix, int* iy) const {
        *ix = static_cast<int>(std::floor(p.x() / options_.tile_size_m));
        *iy = static_cast<int>(std::floor(p.y() / options_.tile_size_m));
    }

    Tile& EnsureTileLocked(int ix, int iy) {
        const Key k = PackKey(ix, iy);
        auto it = tiles_.find(k);
        if (it != tiles_.end()) {
            return it->second;
        }
        Tile tile(options_.ivox);
        tile.key = k;
        tile.ix = ix;
        tile.iy = iy;
        auto [ins, _] = tiles_.emplace(k, std::move(tile));
        return ins->second;
    }

    Options options_;
    mutable std::mutex mtx_;
    std::unordered_map<Key, Tile> tiles_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
