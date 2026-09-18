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

#include "autonomy/localization/atlas/sensor/lidar/lightning/ivox/ivox.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>

namespace autonomy::localization::atlas {
namespace sensor {
namespace lightning {

IVox::IVox() : IVox(Options{}) {}

IVox::IVox(Options options) : options_(std::move(options)) {
    if (options_.resolution < 1e-3) {
        options_.resolution = 1e-3;
    }
}

void IVox::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    voxels_.clear();
    num_points_ = 0;
}

std::size_t IVox::num_voxels() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return voxels_.size();
}

std::size_t IVox::num_points() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return num_points_;
}

IVox::Key IVox::ToKey(int ix, int iy, int iz) const {
    // Pack signed 21-bit axes into 63 bits.
    constexpr std::int64_t kMask = (1LL << 21) - 1;
    const auto ux = static_cast<std::int64_t>(ix) & kMask;
    const auto uy = static_cast<std::int64_t>(iy) & kMask;
    const auto uz = static_cast<std::int64_t>(iz) & kMask;
    return (ux << 42) | (uy << 21) | uz;
}

void IVox::IndexOf(const Vec3_t& p, int* ix, int* iy, int* iz) const {
    *ix = static_cast<int>(std::floor(p.x() / options_.resolution));
    *iy = static_cast<int>(std::floor(p.y() / options_.resolution));
    *iz = static_cast<int>(std::floor(p.z() / options_.resolution));
}

void IVox::InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        int ix = 0, iy = 0, iz = 0;
        IndexOf(p, &ix, &iy, &iz);
        auto& voxel = voxels_[ToKey(ix, iy, iz)];
        if (voxel.points.size() >= options_.max_points_per_voxel) {
            continue;
        }
        voxel.points.push_back(p);
        ++num_points_;
        if (voxels_.size() > options_.max_voxels) {
            // Drop an arbitrary bucket to bound memory (FIFO-ish: erase begin).
            auto it = voxels_.begin();
            num_points_ -= it->second.points.size();
            voxels_.erase(it);
        }
    }
}

void IVox::CollectNeighbors(int ix, int iy, int iz,
                            std::vector<Vec3_t>* out) const {
    const int r = options_.neighbor_search;
    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dz = -r; dz <= r; ++dz) {
                const auto it = voxels_.find(ToKey(ix + dx, iy + dy, iz + dz));
                if (it == voxels_.end()) {
                    continue;
                }
                out->insert(out->end(), it->second.points.begin(),
                            it->second.points.end());
            }
        }
    }
}

IVox::PlaneHit IVox::EstimatePlane(const Vec3_t& query_world) const {
    PlaneHit hit;
    if (!query_world.allFinite()) {
        return hit;
    }

    std::vector<Vec3_t> neighbors;
    neighbors.reserve(64);
    {
        std::lock_guard<std::mutex> lock(mtx_);
        int ix = 0, iy = 0, iz = 0;
        IndexOf(query_world, &ix, &iy, &iz);
        CollectNeighbors(ix, iy, iz, &neighbors);
    }

    if (static_cast<int>(neighbors.size()) < options_.min_plane_points) {
        return hit;
    }

    Vec3_t mean = Vec3_t::Zero();
    for (const auto& p : neighbors) {
        mean += p;
    }
    mean /= static_cast<double>(neighbors.size());

    Mat33_t cov = Mat33_t::Zero();
    for (const auto& p : neighbors) {
        const Vec3_t d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(neighbors.size());

    Eigen::SelfAdjointEigenSolver<Mat33_t> solver(cov);
    if (solver.info() != Eigen::Success) {
        return hit;
    }
    // Smallest eigenvalue → plane normal.
    hit.normal = solver.eigenvectors().col(0);
    if (hit.normal.norm() < 1e-9) {
        return hit;
    }
    hit.normal.normalize();
    hit.d = -hit.normal.dot(mean);
    hit.residual = hit.normal.dot(query_world) + hit.d;
    hit.ok = true;
    return hit;
}

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas
