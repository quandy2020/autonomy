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

#include "autonomy/localization/atlas/mapping/tiled_map.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unordered_map>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace mapping {
namespace {

namespace fs = std::filesystem;

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

PclCloud::Ptr ToPcl(const std::vector<Vec3_t>& pts) {
    auto cloud = std::make_shared<PclCloud>();
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        if (!p.allFinite()) {
            continue;
        }
        cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x()),
                                       static_cast<float>(p.y()),
                                       static_cast<float>(p.z())));
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

std::vector<Vec3_t> FromPcl(const PclCloud& cloud) {
    std::vector<Vec3_t> out;
    out.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        out.emplace_back(p.x, p.y, p.z);
    }
    return out;
}

PclCloud::Ptr VoxelDown(const PclCloud::Ptr& in, float leaf) {
    if (!in || in->empty()) {
        return in;
    }
    leaf = std::max(leaf, 0.01f);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    auto out = std::make_shared<PclCloud>();
    vg.filter(*out);
    return out;
}

std::string ChunkFileName(int ix, int iy) {
    return "chunk_" + std::to_string(ix) + "_" + std::to_string(iy) + ".pcd";
}

}  // namespace

TiledMap::TiledMap(Options options) : options_(std::move(options)) {
    if (options_.tile_size_m < 1.0) {
        options_.tile_size_m = 1.0;
    }
    if (options_.voxel_size_m < 1e-3) {
        options_.voxel_size_m = 0.05;
    }
    if (options_.occupancy_cell_m < 1e-3) {
        options_.occupancy_cell_m = 0.5;
    }
}

TiledMap::Key TiledMap::PackKey(int ix, int iy) {
    return (static_cast<Key>(ix) << 32) ^
           static_cast<Key>(static_cast<std::uint32_t>(iy));
}

void TiledMap::IndexOf(const Vec3_t& p, int* ix, int* iy) const {
    *ix = static_cast<int>(std::floor(p.x() / options_.tile_size_m));
    *iy = static_cast<int>(std::floor(p.y() / options_.tile_size_m));
}

TiledMap::Key TiledMap::ChunkKeyFromXYZ(double x, double y, double /*z*/) const {
    int ix = 0;
    int iy = 0;
    IndexOf(Vec3_t(x, y, 0.0), &ix, &iy);
    return PackKey(ix, iy);
}

TiledMap::Key TiledMap::ChunkKeyFromPose(const Mat44_t& T_wb) const {
    const Vec3_t t = T_wb.block<3, 1>(0, 3);
    return ChunkKeyFromXYZ(t.x(), t.y(), t.z());
}

Vec3_t TiledMap::TileCenterEstimate(int ix, int iy) const {
    const double s = options_.tile_size_m;
    return Vec3_t((static_cast<double>(ix) + 0.5) * s,
                  (static_cast<double>(iy) + 0.5) * s, 0.0);
}

TiledMap::Key TiledMap::OccupancyKey(int ix, int iy, const Vec3_t& p) const {
    const double origin_x = static_cast<double>(ix) * options_.tile_size_m;
    const double origin_y = static_cast<double>(iy) * options_.tile_size_m;
    const int cx =
        static_cast<int>(std::floor((p.x() - origin_x) / options_.occupancy_cell_m));
    const int cy =
        static_cast<int>(std::floor((p.y() - origin_y) / options_.occupancy_cell_m));
    return PackKey(cx, cy);
}

TiledMap::Tile& TiledMap::EnsureTileLocked(int ix, int iy) {
    const Key k = PackKey(ix, iy);
    auto [it, inserted] = tiles_.try_emplace(k, options_.ivox);
    if (inserted) {
        it->second.key = k;
        it->second.ix = ix;
        it->second.iy = iy;
        it->second.filename = ChunkFileName(ix, iy);
        it->second.loaded = true;
    }
    return it->second;
}

void TiledMap::AddPointLocked(Tile& tile, const Vec3_t& p) {
    if (options_.use_ivox_per_tile) {
        if (tile.ivox) {
            tile.ivox->InsertWorldPoints({p});
        }
    } else {
        if (tile.points.size() >= options_.max_points_per_tile) {
            return;
        }
        tile.points.push_back(p);
    }
    tile.ExpandBound(p);
    tile.loaded = true;
    if (options_.store_occupancy) {
        ++tile.occupancy[OccupancyKey(tile.ix, tile.iy, p)];
    }
}

void TiledMap::DownsampleTileLocked(Tile& tile) {
    if (options_.use_ivox_per_tile || tile.points.size() < 32) {
        return;
    }
    auto cloud = ToPcl(tile.points);
    cloud = VoxelDown(cloud, static_cast<float>(options_.voxel_size_m));
    tile.points = FromPcl(*cloud);
    if (tile.points.size() > options_.max_points_per_tile) {
        tile.points.resize(options_.max_points_per_tile);
    }
    tile.min_bound = Vec3_t::Constant(1e30);
    tile.max_bound = Vec3_t::Constant(-1e30);
    for (const auto& p : tile.points) {
        tile.ExpandBound(p);
    }
}

int TiledMap::IntegrateScan(const Mat44_t& T_wb,
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

int TiledMap::InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
    std::lock_guard<std::mutex> lock(mtx_);
    int n = 0;
    std::unordered_map<Key, bool> touched;
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        int ix = 0;
        int iy = 0;
        IndexOf(p, &ix, &iy);
        Tile& tile = EnsureTileLocked(ix, iy);
        const std::size_t before = tile.points.size();
        AddPointLocked(tile, p);
        if (options_.use_ivox_per_tile || tile.points.size() > before) {
            ++n;
            touched[tile.key] = true;
        }
    }
    for (const auto& kv : touched) {
        auto it = tiles_.find(kv.first);
        if (it != tiles_.end() &&
            it->second.points.size() > options_.max_points_per_tile / 2) {
            DownsampleTileLocked(it->second);
        }
    }
    return n;
}

bool TiledMap::Save(const std::string& dir) const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        LOG(ERROR) << "TiledMap::Save: cannot create dir " << dir << ": "
                   << ec.message();
        return false;
    }

    YAML::Node root;
    root["tile_size_m"] = options_.tile_size_m;
    root["voxel_size_m"] = options_.voxel_size_m;
    root["binary_compressed"] = options_.save_binary_compressed;
    root["store_occupancy"] = options_.store_occupancy;
    root["occupancy_cell_m"] = options_.occupancy_cell_m;
    root["num_tiles"] = static_cast<int>(tiles_.size());

    YAML::Node tiles_node(YAML::NodeType::Sequence);
    int saved = 0;
    for (const auto& kv : tiles_) {
        const Tile& tile = kv.second;
        std::vector<Vec3_t> pts = tile.points;
        if (options_.use_ivox_per_tile && pts.empty()) {
            // IVox path: skip empty vector tiles (no PCD dump of ivox yet).
            continue;
        }
        if (pts.empty() && !tile.loaded) {
            continue;
        }

        auto cloud = ToPcl(pts);
        if (!cloud->empty()) {
            cloud = VoxelDown(cloud, static_cast<float>(options_.voxel_size_m));
        }
        const std::string rel =
            tile.filename.empty() ? ChunkFileName(tile.ix, tile.iy)
                                  : tile.filename;
        const fs::path abs = fs::path(dir) / rel;
        if (!cloud->empty()) {
            cloud->width = static_cast<std::uint32_t>(cloud->size());
            cloud->height = 1;
            const int rc =
                options_.save_binary_compressed
                    ? pcl::io::savePCDFileBinaryCompressed(abs.string(), *cloud)
                    : pcl::io::savePCDFileASCII(abs.string(), *cloud);
            if (rc != 0) {
                LOG(ERROR) << "TiledMap::Save: failed PCD " << abs;
                return false;
            }
        }

        YAML::Node entry;
        entry["key"] = static_cast<long long>(tile.key);
        entry["ix"] = tile.ix;
        entry["iy"] = tile.iy;
        entry["file"] = rel;
        entry["num_points"] = static_cast<int>(cloud ? cloud->size() : 0);
        entry["min"] = YAML::Node(YAML::NodeType::Sequence);
        entry["min"].push_back(tile.min_bound.x());
        entry["min"].push_back(tile.min_bound.y());
        entry["min"].push_back(tile.min_bound.z());
        entry["max"] = YAML::Node(YAML::NodeType::Sequence);
        entry["max"].push_back(tile.max_bound.x());
        entry["max"].push_back(tile.max_bound.y());
        entry["max"].push_back(tile.max_bound.z());
        if (options_.store_occupancy && !tile.occupancy.empty()) {
            entry["occupancy_cells"] = static_cast<int>(tile.occupancy.size());
        }
        tiles_node.push_back(entry);
        ++saved;
    }
    root["tiles"] = tiles_node;

    const fs::path index_path = fs::path(dir) / "index.yaml";
    std::ofstream ofs(index_path);
    if (!ofs) {
        LOG(ERROR) << "TiledMap::Save: cannot write " << index_path;
        return false;
    }
    ofs << root;
    LOG(INFO) << "TiledMap::Save: " << saved << " tiles → " << dir;
    return true;
}

bool TiledMap::Load(const std::string& dir) {
    const fs::path index_path = fs::path(dir) / "index.yaml";
    YAML::Node root;
    try {
        root = YAML::LoadFile(index_path.string());
    } catch (const std::exception& e) {
        LOG(ERROR) << "TiledMap::Load: cannot read " << index_path << ": "
                   << e.what();
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    tiles_.clear();
    map_dir_ = dir;

    if (root["tile_size_m"]) {
        options_.tile_size_m = root["tile_size_m"].as<double>(options_.tile_size_m);
    }
    if (root["voxel_size_m"]) {
        options_.voxel_size_m =
            root["voxel_size_m"].as<double>(options_.voxel_size_m);
    }
    if (root["binary_compressed"]) {
        options_.save_binary_compressed =
            root["binary_compressed"].as<bool>(options_.save_binary_compressed);
    }
    if (root["store_occupancy"]) {
        options_.store_occupancy =
            root["store_occupancy"].as<bool>(options_.store_occupancy);
    }
    if (root["occupancy_cell_m"]) {
        options_.occupancy_cell_m =
            root["occupancy_cell_m"].as<double>(options_.occupancy_cell_m);
    }

    const YAML::Node tiles_node = root["tiles"];
    if (!tiles_node || !tiles_node.IsSequence()) {
        LOG(WARNING) << "TiledMap::Load: empty tiles in " << index_path;
        return true;
    }

    int loaded = 0;
    for (const auto& entry : tiles_node) {
        const int ix = entry["ix"].as<int>(0);
        const int iy = entry["iy"].as<int>(0);
        const Key k = PackKey(ix, iy);
        auto [it, inserted] = tiles_.try_emplace(k, options_.ivox);
        Tile& tile = it->second;
        tile.key = k;
        tile.ix = ix;
        tile.iy = iy;
        tile.filename = entry["file"].as<std::string>(ChunkFileName(ix, iy));
        if (entry["min"] && entry["min"].size() >= 3) {
            tile.min_bound =
                Vec3_t(entry["min"][0].as<double>(), entry["min"][1].as<double>(),
                       entry["min"][2].as<double>());
        }
        if (entry["max"] && entry["max"].size() >= 3) {
            tile.max_bound =
                Vec3_t(entry["max"][0].as<double>(), entry["max"][1].as<double>(),
                       entry["max"][2].as<double>());
        }
        if (!LoadTilePcdLocked(tile)) {
            LOG(WARNING) << "TiledMap::Load: skip missing PCD " << tile.filename;
            tile.loaded = false;
        }
        (void)inserted;
        ++loaded;
    }
    LOG(INFO) << "TiledMap::Load: " << loaded << " tiles from " << dir;
    return true;
}

bool TiledMap::LoadTilePcdLocked(Tile& tile) {
    if (map_dir_.empty() || tile.filename.empty()) {
        return false;
    }
    const fs::path abs = fs::path(map_dir_) / tile.filename;
    if (!fs::exists(abs)) {
        return false;
    }
    PclCloud cloud;
    if (pcl::io::loadPCDFile(abs.string(), cloud) != 0) {
        return false;
    }
    tile.points = FromPcl(cloud);
    tile.min_bound = Vec3_t::Constant(1e30);
    tile.max_bound = Vec3_t::Constant(-1e30);
    for (const auto& p : tile.points) {
        tile.ExpandBound(p);
        if (options_.store_occupancy) {
            ++tile.occupancy[OccupancyKey(tile.ix, tile.iy, p)];
        }
        if (options_.use_ivox_per_tile) {
            if (tile.ivox) {
                tile.ivox->InsertWorldPoints({p});
            }
        }
    }
    tile.loaded = true;
    return true;
}

void TiledMap::UnloadTileLocked(Tile& tile) {
    tile.points.clear();
    tile.points.shrink_to_fit();
    tile.occupancy.clear();
    if (options_.use_ivox_per_tile && tile.ivox) {
        tile.ivox->Clear();
    }
    tile.loaded = false;
}

void TiledMap::LoadOnPose(const Vec3_t& pos, double radius) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (radius < 0.0) {
        radius = 0.0;
    }
    const double unload_r = radius + std::max(0.0, options_.unload_margin_m);
    const double r2 = radius * radius;
    const double unload_r2 = unload_r * unload_r;

    for (auto& kv : tiles_) {
        Tile& tile = kv.second;
        Vec3_t center = tile.loaded && tile.min_bound.x() <= tile.max_bound.x()
                            ? tile.Center()
                            : TileCenterEstimate(tile.ix, tile.iy);
        // Keep z of pose for distance in XY mainly (map tiles are XY).
        center.z() = pos.z();
        const double d2 = (center - pos).squaredNorm();
        if (d2 <= r2) {
            if (!tile.loaded) {
                if (!LoadTilePcdLocked(tile)) {
                    // No on-disk file yet (live tile never saved) — keep empty.
                    tile.loaded = true;
                }
            }
        } else if (options_.unload_far_tiles && d2 > unload_r2 && tile.loaded) {
            UnloadTileLocked(tile);
        }
    }
}

void TiledMap::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    tiles_.clear();
    map_dir_.clear();
}

std::size_t TiledMap::num_tiles() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return tiles_.size();
}

std::size_t TiledMap::num_loaded_tiles() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::size_t n = 0;
    for (const auto& kv : tiles_) {
        if (kv.second.loaded) {
            ++n;
        }
    }
    return n;
}

std::size_t TiledMap::num_points() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::size_t n = 0;
    for (const auto& kv : tiles_) {
        n += kv.second.points.size();
    }
    return n;
}

}  // namespace mapping
}  // namespace autonomy::localization::atlas
