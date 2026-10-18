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

#include "argus/relocalization/prior_map_loader.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

namespace argus {
namespace relocalization {

PriorOccupancyMap::PriorOccupancyMap(double resolution_m)
    : resolution_(resolution_m > 1e-6 ? resolution_m : 0.5) {}

void PriorOccupancyMap::Clear() {
    occupied_.clear();
    has_bounds_ = false;
    min_bound_ = Eigen::Vector3d::Constant(1e9);
    max_bound_ = Eigen::Vector3d::Constant(-1e9);
}

void PriorOccupancyMap::VoxelIndex(const Eigen::Vector3d& p, int* ix, int* iy,
                                   int* iz) const {
    *ix = static_cast<int>(std::floor(p.x() / resolution_));
    *iy = static_cast<int>(std::floor(p.y() / resolution_));
    *iz = static_cast<int>(std::floor(p.z() / resolution_));
}

int64_t PriorOccupancyMap::MakeVoxelKey(int ix, int iy, int iz) const {
    // 21 bits per axis packing (adequate for campus-scale maps).
    constexpr int64_t kBias = 1 << 20;
    return ((static_cast<int64_t>(ix) + kBias) << 42) |
           ((static_cast<int64_t>(iy) + kBias) << 21) |
           (static_cast<int64_t>(iz) + kBias);
}

void PriorOccupancyMap::InsertPoint(const Eigen::Vector3d& p_w) {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    VoxelIndex(p_w, &ix, &iy, &iz);
    occupied_.insert(MakeVoxelKey(ix, iy, iz));
    if (!has_bounds_) {
        min_bound_ = max_bound_ = p_w;
        has_bounds_ = true;
    } else {
        min_bound_ = min_bound_.cwiseMin(p_w);
        max_bound_ = max_bound_.cwiseMax(p_w);
    }
}

void PriorOccupancyMap::InsertCloud(const sensor::PointCloudXYZI& cloud_w) {
    for (std::size_t i = 0; i < cloud_w.size(); ++i) {
        InsertPoint(Eigen::Vector3d(cloud_w[i].x, cloud_w[i].y, cloud_w[i].z));
    }
}

bool PriorOccupancyMap::Occupied(const Eigen::Vector3d& p_w) const {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    VoxelIndex(p_w, &ix, &iy, &iz);
    return occupied_.count(MakeVoxelKey(ix, iy, iz)) > 0;
}

double PriorOccupancyMap::Score(const sensor::PointCloudXYZI& cloud_b,
                                const transform::Rigid3d& T_w_b) const {
    if (cloud_b.empty() || occupied_.empty()) {
        return 0.0;
    }
    int hits = 0;
    for (std::size_t i = 0; i < cloud_b.size(); ++i) {
        const Eigen::Vector3d pb(cloud_b[i].x, cloud_b[i].y, cloud_b[i].z);
        if (Occupied(T_w_b * pb)) {
            ++hits;
        }
    }
    return static_cast<double>(hits) / static_cast<double>(cloud_b.size());
}

PriorOccupancyMap::SharedPtr PriorMapLoader::FromCloud(
    const sensor::PointCloudXYZI& cloud_w, double resolution_m) {
    auto map = PriorOccupancyMap::make_shared(resolution_m);
    map->InsertCloud(cloud_w);
    return map;
}

PriorOccupancyMap::SharedPtr PriorMapLoader::LoadXyz(const std::string& path,
                                                     double resolution_m) {
    std::ifstream in(path);
    if (!in) {
        return nullptr;
    }
    auto map = PriorOccupancyMap::make_shared(resolution_m);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        double x = 0;
        double y = 0;
        double z = 0;
        if (!(ss >> x >> y >> z)) {
            continue;
        }
        map->InsertPoint(Eigen::Vector3d(x, y, z));
    }
    if (map->size() == 0) {
        return nullptr;
    }
    return map;
}

PriorOccupancyMap::SharedPtr PriorMapLoader::LoadPath(
    const std::string& path_or_dir, double resolution_m) {
    if (path_or_dir.empty()) {
        return nullptr;
    }
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path p(path_or_dir);
    if (fs::is_directory(p, ec)) {
        // Prefer dedicated prior cloud from MapBuilder package v2.
        if (fs::exists(p / "prior.xyz", ec)) {
            auto prior = LoadXyz((p / "prior.xyz").string(), resolution_m);
            if (prior) {
                return prior;
            }
        }
        return LoadXyz((p / "map.xyz").string(), resolution_m);
    }
    return LoadXyz(path_or_dir, resolution_m);
}

}  // namespace relocalization
}  // namespace argus
