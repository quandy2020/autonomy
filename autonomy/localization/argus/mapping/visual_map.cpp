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

#include "argus/map/visual_map.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace argus {
namespace map {

VisualMap::VisualMap(double voxel_size) : voxel_size_(voxel_size) {}

VisualMap::~VisualMap() {
    Clear();
}

void VisualMap::Clear() {
    for (auto& pair : feat_map_) {
        delete pair.second;
    }
    feat_map_.clear();
}

VoxelLocation VisualMap::KeyForPoint(const Eigen::Vector3d& pt_w) const {
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = pt_w[j] / voxel_size_;
        if (loc_xyz[j] < 0) {
            loc_xyz[j] -= 1.0;
        }
    }
    return VoxelLocation(static_cast<int64_t>(loc_xyz[0]),
                         static_cast<int64_t>(loc_xyz[1]),
                         static_cast<int64_t>(loc_xyz[2]));
}

void VisualMap::InsertPoint(VisualPoint* pt_new) {
    if (pt_new == nullptr) {
        return;
    }
    const VoxelLocation position = KeyForPoint(pt_new->pos_);
    auto iter = feat_map_.find(position);
    if (iter != feat_map_.end()) {
        iter->second->voxel_points.push_back(pt_new);
        iter->second->count++;
    } else {
        auto* ot = new VoxelPoints(0);
        ot->voxel_points.push_back(pt_new);
        ot->count++;
        feat_map_[position] = ot;
    }
}

std::size_t VisualMap::LandmarkCount() const {
    std::size_t n = 0;
    for (const auto& pair : feat_map_) {
        if (pair.second != nullptr) {
            n += pair.second->voxel_points.size();
        }
    }
    return n;
}

bool VisualMap::SaveLandmarks(const std::string& path) const {
    std::ofstream os(path);
    if (!os) {
        return false;
    }
    os << "# visual_landmarks v1 x y z nx ny nz\n";
    for (const auto& pair : feat_map_) {
        if (pair.second == nullptr) {
            continue;
        }
        for (const VisualPoint* vp : pair.second->voxel_points) {
            if (vp == nullptr) {
                continue;
            }
            os << vp->pos_.x() << ' ' << vp->pos_.y() << ' ' << vp->pos_.z()
               << ' ' << vp->normal_.x() << ' ' << vp->normal_.y() << ' '
               << vp->normal_.z() << '\n';
        }
    }
    return static_cast<bool>(os);
}

bool VisualMap::LoadLandmarks(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    Clear();
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        double x = 0;
        double y = 0;
        double z = 0;
        double nx = 0;
        double ny = 0;
        double nz = 0;
        std::istringstream ss(line);
        if (!(ss >> x >> y >> z)) {
            continue;
        }
        ss >> nx >> ny >> nz;  // normals optional
        auto* vp = new VisualPoint(Eigen::Vector3d(x, y, z));
        vp->normal_ = Eigen::Vector3d(nx, ny, nz);
        if (vp->normal_.squaredNorm() > 1e-12) {
            vp->is_normal_initialized_ = true;
        }
        InsertPoint(vp);
    }
    return true;
}

}  // namespace map
}  // namespace argus
