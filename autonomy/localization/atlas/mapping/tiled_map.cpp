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
#include <chrono>
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

std::string ChunkDynFileName(int ix, int iy) {
    return "chunk_" + std::to_string(ix) + "_" + std::to_string(iy) +
           "_dyn.pcd";
}

bool SavePcdFile(const fs::path& abs, const std::vector<Vec3_t>& pts,
                 double voxel_size_m, bool binary_compressed) {
    auto cloud = ToPcl(pts);
    if (cloud->empty()) {
        return true;
    }
    cloud = VoxelDown(cloud, static_cast<float>(voxel_size_m));
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    const int rc =
        binary_compressed
            ? pcl::io::savePCDFileBinaryCompressed(abs.string(), *cloud)
            : pcl::io::savePCDFileASCII(abs.string(), *cloud);
    return rc == 0;
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
        if (options_.enable_dyn_layer) {
            it->second.dyn_filename = ChunkDynFileName(ix, iy);
        }
        it->second.loaded = true;
    }
    return it->second;
}

void TiledMap::AddPointLocked(Tile& tile, const Vec3_t& p, TileLayer layer) {
    auto* dest = &tile.points;
    if (layer == TileLayer::kDynamic) {
        dest = &tile.dyn_points;
    }
    if (options_.use_ivox_per_tile && layer != TileLayer::kDynamic) {
        if (tile.ivox) {
            tile.ivox->InsertWorldPoints({p});
        }
    } else {
        if (dest->size() >= options_.max_points_per_tile) {
            return;
        }
        dest->push_back(p);
    }
    tile.ExpandBound(p);
    tile.loaded = true;
    if (options_.store_occupancy && layer != TileLayer::kDynamic) {
        ++tile.occupancy[OccupancyKey(tile.ix, tile.iy, p)];
    }
}

void TiledMap::DownsampleTileLocked(Tile& tile) {
    if (options_.use_ivox_per_tile) {
        return;
    }
    auto downsample = [&](std::vector<Vec3_t>& pts) {
        if (pts.size() < 32) {
            return;
        }
        auto cloud = ToPcl(pts);
        cloud = VoxelDown(cloud, static_cast<float>(options_.voxel_size_m));
        pts = FromPcl(*cloud);
        if (pts.size() > options_.max_points_per_tile) {
            pts.resize(options_.max_points_per_tile);
        }
    };
    downsample(tile.points);
    downsample(tile.dyn_points);
    tile.min_bound = Vec3_t::Constant(1e30);
    tile.max_bound = Vec3_t::Constant(-1e30);
    for (const auto& p : tile.points) {
        tile.ExpandBound(p);
    }
    for (const auto& p : tile.dyn_points) {
        tile.ExpandBound(p);
    }
}

int TiledMap::IntegrateScan(const Mat44_t& T_wb,
                            const std::vector<Vec3_t>& points_body,
                            double t_sec) {
    if (points_body.empty()) {
        return 0;
    }
    double now = t_sec;
    if (now <= 0.0) {
        const auto tp = std::chrono::steady_clock::now().time_since_epoch();
        now = std::chrono::duration<double>(tp).count();
    }
    const Mat33_t R = T_wb.block<3, 3>(0, 0);
    const Vec3_t t = T_wb.block<3, 1>(0, 3);
    std::vector<Vec3_t> world_static;
    std::vector<Vec3_t> world_dyn;
    world_static.reserve(points_body.size());
    if (options_.enable_dyn_layer) {
        world_dyn.reserve(points_body.size() / 8);
    }
    for (const auto& p : points_body) {
        if (!p.allFinite()) {
            continue;
        }
        const Vec3_t pw = R * p + t;
        if (options_.enable_dyn_layer && p.z() > options_.dyn_body_z_thresh) {
            world_dyn.push_back(pw);
        } else {
            world_static.push_back(pw);
        }
    }
    int n = InsertWorldPoints(world_static, TileLayer::kStatic);
    if (!world_dyn.empty()) {
        n += InsertWorldPoints(world_dyn, TileLayer::kDynamic);
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto& kv : tiles_) {
            if (!kv.second.dyn_points.empty()) {
                kv.second.dyn_last_insert_t = now;
            }
        }
    }
    (void)PruneDyn(now);
    return n;
}

int TiledMap::UpdateDynamicCloud(const std::vector<Vec3_t>& points_world,
                                 double t_sec, bool /*remove_old*/) {
    if (points_world.empty()) {
        return 0;
    }
    if (!options_.enable_dyn_layer) {
        options_.enable_dyn_layer = true;
    }
    double now = t_sec;
    if (now <= 0.0) {
        const auto tp = std::chrono::steady_clock::now().time_since_epoch();
        now = std::chrono::duration<double>(tp).count();
    }
    const int n = InsertWorldPoints(points_world, TileLayer::kDynamic);
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto& kv : tiles_) {
            if (!kv.second.dyn_points.empty()) {
                kv.second.dyn_last_insert_t = now;
            }
        }
    }
    (void)PruneDyn(now);
    return n;
}

int TiledMap::PruneDyn(double now_sec) {
    if (!options_.enable_dyn_layer ||
        options_.dyn_policy == DynPolicy::kPermanent) {
        return 0;
    }
    const double ttl = (options_.dyn_policy == DynPolicy::kShort)
                           ? options_.dyn_short_ttl_s
                           : options_.dyn_medium_ttl_s;
    if (ttl <= 0.0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    int removed = 0;
    for (auto& kv : tiles_) {
        auto& tile = kv.second;
        if (tile.dyn_points.empty()) {
            continue;
        }
        if (now_sec - tile.dyn_last_insert_t > ttl) {
            removed += static_cast<int>(tile.dyn_points.size());
            tile.dyn_points.clear();
            tile.dyn_points.shrink_to_fit();
            tile.dyn_last_insert_t = 0.0;
        }
    }
    return removed;
}

int TiledMap::InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
    return InsertWorldPoints(points_world, TileLayer::kStatic);
}

int TiledMap::InsertWorldPoints(const std::vector<Vec3_t>& points_world,
                                TileLayer layer) {
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
        auto& dest =
            (layer == TileLayer::kDynamic) ? tile.dyn_points : tile.points;
        const std::size_t before = dest.size();
        AddPointLocked(tile, p, layer);
        if (options_.use_ivox_per_tile || dest.size() > before) {
            ++n;
            touched[tile.key] = true;
        }
    }
    for (const auto& kv : touched) {
        auto it = tiles_.find(kv.first);
        if (it == tiles_.end()) {
            continue;
        }
        const std::size_t total =
            it->second.points.size() + it->second.dyn_points.size();
        if (total > options_.max_points_per_tile / 2) {
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
    root["enable_dyn_layer"] = options_.enable_dyn_layer;
    root["dyn_body_z_thresh"] = options_.dyn_body_z_thresh;
    root["dyn_policy"] = DynPolicyName(options_.dyn_policy);
    root["dyn_short_ttl_s"] = options_.dyn_short_ttl_s;
    root["dyn_medium_ttl_s"] = options_.dyn_medium_ttl_s;
    root["layers"] = options_.enable_dyn_layer ? "static+dynamic" : "static";
    root["num_tiles"] = static_cast<int>(tiles_.size());

    YAML::Node tiles_node(YAML::NodeType::Sequence);
    int saved = 0;
    for (const auto& kv : tiles_) {
        const Tile& tile = kv.second;
        std::vector<Vec3_t> pts = tile.points;
        if (options_.use_ivox_per_tile && pts.empty() &&
            tile.dyn_points.empty()) {
            continue;
        }
        if (pts.empty() && tile.dyn_points.empty() && !tile.loaded) {
            continue;
        }

        const std::string rel =
            tile.filename.empty() ? ChunkFileName(tile.ix, tile.iy)
                                  : tile.filename;
        const fs::path abs = fs::path(dir) / rel;
        if (!SavePcdFile(abs, pts, options_.voxel_size_m,
                         options_.save_binary_compressed)) {
            LOG(ERROR) << "TiledMap::Save: failed PCD " << abs;
            return false;
        }

        std::string dyn_rel = tile.dyn_filename;
        if (options_.enable_dyn_layer && !tile.dyn_points.empty()) {
            if (dyn_rel.empty()) {
                dyn_rel = ChunkDynFileName(tile.ix, tile.iy);
            }
            const fs::path dyn_abs = fs::path(dir) / dyn_rel;
            if (!SavePcdFile(dyn_abs, tile.dyn_points, options_.voxel_size_m,
                             options_.save_binary_compressed)) {
                LOG(ERROR) << "TiledMap::Save: failed dyn PCD " << dyn_abs;
                return false;
            }
        }

        YAML::Node entry;
        entry["key"] = static_cast<long long>(tile.key);
        entry["ix"] = tile.ix;
        entry["iy"] = tile.iy;
        entry["file"] = rel;
        entry["num_points"] = static_cast<int>(pts.size());
        if (!dyn_rel.empty() && !tile.dyn_points.empty()) {
            entry["dyn_file"] = dyn_rel;
            entry["num_dyn_points"] = static_cast<int>(tile.dyn_points.size());
        }
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
    if (root["enable_dyn_layer"]) {
        options_.enable_dyn_layer =
            root["enable_dyn_layer"].as<bool>(options_.enable_dyn_layer);
    }
    if (root["dyn_body_z_thresh"]) {
        options_.dyn_body_z_thresh =
            root["dyn_body_z_thresh"].as<double>(options_.dyn_body_z_thresh);
    }
    if (root["dyn_policy"]) {
        options_.dyn_policy =
            ParseDynPolicy(root["dyn_policy"].as<std::string>("permanent"));
    }
    if (root["dyn_short_ttl_s"]) {
        options_.dyn_short_ttl_s =
            root["dyn_short_ttl_s"].as<double>(options_.dyn_short_ttl_s);
    }
    if (root["dyn_medium_ttl_s"]) {
        options_.dyn_medium_ttl_s =
            root["dyn_medium_ttl_s"].as<double>(options_.dyn_medium_ttl_s);
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
        if (entry["dyn_file"]) {
            tile.dyn_filename = entry["dyn_file"].as<std::string>("");
            options_.enable_dyn_layer = true;
        }
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
    tile.dyn_points.clear();
    if (!tile.dyn_filename.empty()) {
        const fs::path dyn_abs = fs::path(map_dir_) / tile.dyn_filename;
        if (fs::exists(dyn_abs)) {
            PclCloud dyn_cloud;
            if (pcl::io::loadPCDFile(dyn_abs.string(), dyn_cloud) == 0) {
                tile.dyn_points = FromPcl(dyn_cloud);
            }
        }
        // Missing dyn file is OK (backward compatible: all points in points).
    }
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
    for (const auto& p : tile.dyn_points) {
        tile.ExpandBound(p);
    }
    tile.loaded = true;
    return true;
}

void TiledMap::UnloadTileLocked(Tile& tile) {
    tile.points.clear();
    tile.points.shrink_to_fit();
    tile.dyn_points.clear();
    tile.dyn_points.shrink_to_fit();
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

void TiledMap::CollectLoadedPoints(TileLayer layer,
                                   std::vector<Vec3_t>* out) const {
    if (!out) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& kv : tiles_) {
        const Tile& tile = kv.second;
        if (!tile.loaded) {
            continue;
        }
        if (layer == TileLayer::kStatic || layer == TileLayer::kAll) {
            out->insert(out->end(), tile.points.begin(), tile.points.end());
        }
        if (layer == TileLayer::kDynamic || layer == TileLayer::kAll) {
            out->insert(out->end(), tile.dyn_points.begin(),
                        tile.dyn_points.end());
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
        n += kv.second.points.size() + kv.second.dyn_points.size();
    }
    return n;
}

}  // namespace mapping
}  // namespace autonomy::localization::atlas
