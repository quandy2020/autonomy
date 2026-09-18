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

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"

#include "autonomy/localization/atlas/mapping/ivox/hilbert.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace autonomy::localization::atlas {
namespace mapping {
namespace {

inline std::uint64_t ExpandBits21(std::uint32_t v) {
    std::uint64_t x = v & 0x1fffffu;
    x = (x | (x << 32)) & 0x1f00000000ffffull;
    x = (x | (x << 16)) & 0x1f0000ff0000ffull;
    x = (x | (x << 8)) & 0x100f00f00f00f00full;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3ull;
    x = (x | (x << 2)) & 0x1249249249249249ull;
    return x;
}

}  // namespace

IVox::IVox() : IVox(Options{}) {}

IVox::IVox(Options options) : options_(std::move(options)) {
    if (options_.resolution < 1e-3) {
        options_.resolution = 1e-3;
    }
}

void IVox::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    voxels_.clear();
    insert_order_.clear();
    num_points_ = 0;
    stats_ = CapacityStats{};
}

std::size_t IVox::num_voxels() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return voxels_.size();
}

std::size_t IVox::num_points() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return num_points_;
}

IVox::CapacityStats IVox::capacity_stats() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return stats_;
}

IVox::Key IVox::MortonEncode3(int ix, int iy, int iz) {
    // Offset signed axes into unsigned 21-bit range centered at 2^20.
    constexpr int kBias = 1 << 20;
    const auto ux = static_cast<std::uint32_t>(ix + kBias) & 0x1fffffu;
    const auto uy = static_cast<std::uint32_t>(iy + kBias) & 0x1fffffu;
    const auto uz = static_cast<std::uint32_t>(iz + kBias) & 0x1fffffu;
    return static_cast<Key>((ExpandBits21(ux)) | (ExpandBits21(uy) << 1) |
                            (ExpandBits21(uz) << 2));
}

IVox::Key IVox::HilbertEncode3(int ix, int iy, int iz) {
    // Bias signed indices into uint16 range, then Hilbert PositionToIndex.
    constexpr int kBias = 1 << 15;
    auto clamp_u16 = [](int v) -> std::uint16_t {
        const int c = std::clamp(v, 0, 65535);
        return static_cast<std::uint16_t>(c);
    };
    const std::array<std::uint16_t, 3> apos{
        clamp_u16(ix + kBias), clamp_u16(iy + kBias), clamp_u16(iz + kBias)};
    const std::array<std::uint16_t, 3> idx =
        hilbert::v2::PositionToIndex(apos);
    return (static_cast<Key>(idx[0]) << 32) |
           (static_cast<Key>(idx[1]) << 16) | static_cast<Key>(idx[2]);
}

IVox::Key IVox::ToKey(int ix, int iy, int iz) const {
    if (options_.use_hilbert_key) {
        return HilbertEncode3(ix, iy, iz);
    }
    if (options_.use_morton_key) {
        return MortonEncode3(ix, iy, iz);
    }
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

void IVox::EvictOldestLocked() {
    while (!insert_order_.empty() && voxels_.size() > options_.max_voxels) {
        const Key old = insert_order_.front();
        insert_order_.pop_front();
        const auto it = voxels_.find(old);
        if (it == voxels_.end()) {
            continue;
        }
        num_points_ -= it->second.points.size();
        voxels_.erase(it);
        ++stats_.num_evictions;
    }
}

void IVox::InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        int ix = 0, iy = 0, iz = 0;
        IndexOf(p, &ix, &iy, &iz);
        const Key key = ToKey(ix, iy, iz);
        const bool is_new = (voxels_.find(key) == voxels_.end());
        auto& voxel = voxels_[key];
        if (is_new) {
            insert_order_.push_back(key);
        }
        if (voxel.points.size() >= options_.max_points_per_voxel) {
            EvictOldestLocked();
            continue;
        }
        voxel.points.push_back(p);
        ++num_points_;
        EvictOldestLocked();
    }
}

void IVox::CollectNeighbors(int ix, int iy, int iz,
                            std::vector<Vec3_t>* out,
                            std::size_t* voxel_hits) const {
    std::unordered_set<Key> visited;
    const auto add_voxel = [&](int x, int y, int z) {
        const Key k = ToKey(x, y, z);
        if (!visited.insert(k).second) {
            return;
        }
        const auto it = voxels_.find(k);
        if (it == voxels_.end()) {
            return;
        }
        if (voxel_hits) {
            ++(*voxel_hits);
        }
        out->insert(out->end(), it->second.points.begin(),
                    it->second.points.end());
    };

    // Face-adjacent (6-neigh) for plane robustness.
    if (options_.face_adjacent_neighbors) {
        static const int kFace[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        add_voxel(ix, iy, iz);
        for (const auto& d : kFace) {
            add_voxel(ix + d[0], iy + d[1], iz + d[2]);
        }
    }

    const int r = options_.neighbor_search;
    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dz = -r; dz <= r; ++dz) {
                add_voxel(ix + dx, iy + dy, iz + dz);
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
    std::size_t voxel_hits = 0;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        int ix = 0, iy = 0, iz = 0;
        IndexOf(query_world, &ix, &iy, &iz);
        CollectNeighbors(ix, iy, iz, &neighbors, &voxel_hits);
        stats_.last_neighbor_points = neighbors.size();
        stats_.last_neighbor_voxels = voxel_hits;
        stats_.peak_neighbor_points =
            std::max(stats_.peak_neighbor_points, neighbors.size());
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

}  // namespace mapping
}  // namespace autonomy::localization::atlas
