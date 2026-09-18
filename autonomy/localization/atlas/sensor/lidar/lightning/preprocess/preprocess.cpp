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

#include "autonomy/localization/atlas/sensor/lidar/lightning/preprocess/preprocess.hpp"

#include <cmath>
#include <unordered_set>

namespace autonomy::localization::atlas {
namespace sensor {
namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        return (static_cast<std::size_t>(k.x) * 73856093u) ^
               (static_cast<std::size_t>(k.y) * 19349663u) ^
               (static_cast<std::size_t>(k.z) * 83492791u);
    }
};

}  // namespace

Preprocess::Options Preprocess::FromYaml(const YAML::Node& node) {
    Options o;
    if (!node || !node.IsMap()) {
        return o;
    }
    o.min_range = node["min_range"].as<double>(o.min_range);
    o.max_range = node["max_range"].as<double>(o.max_range);
    o.blind = node["blind"].as<double>(o.blind);
    o.voxel_leaf = node["voxel_leaf"].as<double>(o.voxel_leaf);
    o.max_points = node["max_points"].as<int>(o.max_points);
    o.use_point_time = node["use_point_time"].as<bool>(o.use_point_time);
    return o;
}

std::vector<Vec3_t> Preprocess::Run(
    const std::vector<Vec3_t>& points_body) const {
    std::vector<Vec3_t> filtered;
    filtered.reserve(points_body.size());
    const double min_r2 = options_.min_range * options_.min_range;
    const double max_r2 = options_.max_range * options_.max_range;
    const double blind2 = options_.blind * options_.blind;

    for (const auto& p : points_body) {
        if (!p.allFinite()) {
            continue;
        }
        const double r2 = p.squaredNorm();
        if (r2 < min_r2 || r2 > max_r2 || r2 < blind2) {
            continue;
        }
        filtered.push_back(p);
    }

    if (options_.voxel_leaf <= 1e-6 || filtered.empty()) {
        if (static_cast<int>(filtered.size()) > options_.max_points) {
            filtered.resize(static_cast<std::size_t>(options_.max_points));
        }
        return filtered;
    }

    const double inv = 1.0 / options_.voxel_leaf;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    seen.reserve(filtered.size());
    std::vector<Vec3_t> down;
    down.reserve(std::min(filtered.size(),
                           static_cast<std::size_t>(options_.max_points)));
    for (const auto& p : filtered) {
        VoxelKey key;
        key.x = static_cast<int>(std::floor(p.x() * inv));
        key.y = static_cast<int>(std::floor(p.y() * inv));
        key.z = static_cast<int>(std::floor(p.z() * inv));
        if (!seen.insert(key).second) {
            continue;
        }
        down.push_back(p);
        if (static_cast<int>(down.size()) >= options_.max_points) {
            break;
        }
    }
    return down;
}

std::vector<TimedPoint> Preprocess::RunTimed(
    const std::vector<Vec3_t>& points_body,
    const std::vector<double>& point_time_rel) const {
    const bool have_times = point_time_rel.size() == points_body.size();
    std::vector<TimedPoint> filtered;
    filtered.reserve(points_body.size());
    const double min_r2 = options_.min_range * options_.min_range;
    const double max_r2 = options_.max_range * options_.max_range;
    const double blind2 = options_.blind * options_.blind;

    for (std::size_t i = 0; i < points_body.size(); ++i) {
        const auto& p = points_body[i];
        if (!p.allFinite()) {
            continue;
        }
        const double r2 = p.squaredNorm();
        if (r2 < min_r2 || r2 > max_r2 || r2 < blind2) {
            continue;
        }
        TimedPoint tp;
        tp.p = p;
        tp.t_rel = have_times ? point_time_rel[i] : 0.0;
        filtered.push_back(tp);
    }

    if (options_.voxel_leaf <= 1e-6 || filtered.empty()) {
        if (static_cast<int>(filtered.size()) > options_.max_points) {
            filtered.resize(static_cast<std::size_t>(options_.max_points));
        }
        return filtered;
    }

    const double inv = 1.0 / options_.voxel_leaf;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    seen.reserve(filtered.size());
    std::vector<TimedPoint> down;
    down.reserve(std::min(filtered.size(),
                           static_cast<std::size_t>(options_.max_points)));
    for (const auto& tp : filtered) {
        VoxelKey key;
        key.x = static_cast<int>(std::floor(tp.p.x() * inv));
        key.y = static_cast<int>(std::floor(tp.p.y() * inv));
        key.z = static_cast<int>(std::floor(tp.p.z() * inv));
        if (!seen.insert(key).second) {
            continue;
        }
        down.push_back(tp);
        if (static_cast<int>(down.size()) >= options_.max_points) {
            break;
        }
    }
    return down;
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
