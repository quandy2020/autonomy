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

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <variant>

namespace autonomy::localization::atlas {
namespace mapping {

std::size_t IVox::Key3Hash::operator()(const Key3& k) const noexcept {
    // FNV-ish mix of three ints (stable regardless of PackKey mode).
    std::size_t h = 14695981039346656037ull;
    auto mix = [&](int v) {
        h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(v));
        h *= 1099511628211ull;
    };
    mix(k.x);
    mix(k.y);
    mix(k.z);
    return h;
}

IVox::IVox() : IVox(Options{}) {}

IVox::IVox(Options options) {
    set_options(std::move(options));
}

void IVox::set_options(Options options) {
    std::lock_guard<std::mutex> lock(mtx_);
    options_ = std::move(options);
    if (options_.resolution < 1e-3) {
        options_.resolution = 1e-3;
    }
    inv_resolution_ = 1.0 / options_.resolution;
    options_.phc_order = std::clamp(options_.phc_order, 1, 8);
    GenerateNearbyGrids();
}

void IVox::GenerateNearbyGrids() {
    nearby_grids_.clear();
    auto add = [this](int dx, int dy, int dz) {
        nearby_grids_.push_back(Key3{dx, dy, dz});
    };
    switch (options_.nearby_type) {
        case IVoxNearbyType::kCenter:
            add(0, 0, 0);
            break;
        case IVoxNearbyType::kNearby18:
            add(0, 0, 0);
            add(-1, 0, 0);
            add(1, 0, 0);
            add(0, 1, 0);
            add(0, -1, 0);
            add(0, 0, -1);
            add(0, 0, 1);
            add(1, 1, 0);
            add(-1, 1, 0);
            add(1, -1, 0);
            add(-1, -1, 0);
            add(1, 0, 1);
            add(-1, 0, 1);
            add(1, 0, -1);
            add(-1, 0, -1);
            add(0, 1, 1);
            add(0, -1, 1);
            add(0, 1, -1);
            add(0, -1, -1);
            break;
        case IVoxNearbyType::kNearby26:
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        add(dx, dy, dz);
                    }
                }
            }
            break;
        case IVoxNearbyType::kNearby6:
        default:
            add(0, 0, 0);
            add(-1, 0, 0);
            add(1, 0, 0);
            add(0, 1, 0);
            add(0, -1, 0);
            add(0, 0, -1);
            add(0, 0, 1);
            break;
    }
}

void IVox::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    grids_.clear();
    cache_.clear();
    num_points_ = 0;
    stats_ = CapacityStats{};
}

void IVox::ExportWorldPoints(std::vector<Vec3_t>* out,
                             std::size_t max_points) const {
    if (!out) {
        return;
    }
    out->clear();
    std::lock_guard<std::mutex> lock(mtx_);
    if (cache_.empty()) {
        return;
    }
    out->reserve(max_points > 0 ? std::min(max_points, num_points_)
                                : num_points_);
    for (const auto& entry : cache_) {
        std::visit([&](const auto& node) { node.CollectAll(out); },
                   entry.second);
    }
    if (max_points > 0 && out->size() > max_points) {
        // Stride downsample to keep spatial coverage without full shuffle.
        const std::size_t n = out->size();
        const double step =
            static_cast<double>(n) / static_cast<double>(max_points);
        std::vector<Vec3_t> sampled;
        sampled.reserve(max_points);
        for (std::size_t i = 0; i < max_points; ++i) {
            const std::size_t idx = std::min(
                n - 1, static_cast<std::size_t>(static_cast<double>(i) * step));
            sampled.push_back((*out)[idx]);
        }
        *out = std::move(sampled);
    }
}

std::size_t IVox::num_voxels() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return grids_.size();
}

std::size_t IVox::num_points() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return num_points_;
}

IVox::CapacityStats IVox::capacity_stats() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return stats_;
}

IVox::Key3 IVox::Pos2Grid(const Vec3_t& p) const {
    // Lightning uses round; keep floor for signed stability at origin.
    return Key3{static_cast<int>(std::floor(p.x() * inv_resolution_)),
                static_cast<int>(std::floor(p.y() * inv_resolution_)),
                static_cast<int>(std::floor(p.z() * inv_resolution_))};
}

IVox::Node IVox::MakeNode(const Key3& key) const {
    const Vec3_t center((static_cast<double>(key.x) + 0.5) * options_.resolution,
                        (static_cast<double>(key.y) + 0.5) * options_.resolution,
                        (static_cast<double>(key.z) + 0.5) * options_.resolution);
    if (options_.node_kind == IVoxNodeKind::kPhc) {
        return IVoxNodePhc(center, options_.resolution, options_.phc_order);
    }
    return IVoxNodeLinear(center, options_.resolution,
                          options_.max_points_per_voxel);
}

void IVox::TouchLru(CacheIter it) {
    if (it == cache_.begin()) {
        return;
    }
    cache_.splice(cache_.begin(), cache_, it);
    grids_[it->first] = cache_.begin();
}

void IVox::InsertOne(const Vec3_t& p) {
    const Key3 key = Pos2Grid(p);
    auto it = grids_.find(key);
    if (it == grids_.end()) {
        cache_.push_front({key, MakeNode(key)});
        grids_[key] = cache_.begin();
        std::visit([&](auto& node) { node.Insert(p); }, cache_.front().second);
        ++num_points_;
        while (grids_.size() > options_.max_voxels && !cache_.empty()) {
            const Key3 old = cache_.back().first;
            const std::size_t n = std::visit(
                [](const auto& node) { return node.Size(); }, cache_.back().second);
            num_points_ = (n >= num_points_) ? 0 : (num_points_ - n);
            grids_.erase(old);
            cache_.pop_back();
            ++stats_.num_evictions;
        }
        return;
    }
    auto cit = it->second;
    const std::size_t before =
        std::visit([](const auto& node) { return node.Size(); }, cit->second);
    std::visit([&](auto& node) { node.Insert(p); }, cit->second);
    const std::size_t after =
        std::visit([](const auto& node) { return node.Size(); }, cit->second);
    if (after > before) {
        num_points_ += (after - before);
    }
    // Linear ring at capacity: Size unchanged; point replaced in-place.
    TouchLru(cit);
}

void IVox::InsertWorldPoints(const std::vector<Vec3_t>& points_world) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        InsertOne(p);
    }
}

bool IVox::GetClosestPoints(const Vec3_t& query, std::vector<Vec3_t>* closest,
                            int max_num, double max_range) const {
    if (!closest || max_num <= 0) {
        return false;
    }
    closest->clear();

    struct Cand {
        double dist2 = 0.0;
        Vec3_t p = Vec3_t::Zero();
        bool operator<(const Cand& o) const { return dist2 < o.dist2; }
    };
    std::vector<Cand> cands;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        const Key3 key = Pos2Grid(query);
        std::size_t voxel_hits = 0;
        for (const Key3& d : nearby_grids_) {
            const Key3 nk{key.x + d.x, key.y + d.y, key.z + d.z};
            const auto it = grids_.find(nk);
            if (it == grids_.end()) {
                continue;
            }
            ++voxel_hits;
            std::visit(
                [&](const auto& node) {
                    using NodeT = std::decay_t<decltype(node)>;
                    std::vector<typename NodeT::DistPoint> local;
                    node.KNNByCondition(&local, query, max_num, max_range);
                    for (const auto& dp : local) {
                        cands.push_back(Cand{dp.dist2, node.GetPoint(dp.idx)});
                    }
                },
                it->second->second);
        }
        stats_.last_neighbor_voxels = voxel_hits;
        stats_.last_neighbor_points = cands.size();
        stats_.peak_neighbor_points =
            std::max(stats_.peak_neighbor_points, cands.size());
    }

    if (cands.empty()) {
        return false;
    }
    if (static_cast<int>(cands.size()) > max_num) {
        std::nth_element(cands.begin(), cands.begin() + max_num - 1, cands.end());
        cands.resize(static_cast<std::size_t>(max_num));
    }
    std::sort(cands.begin(), cands.end());
    closest->reserve(cands.size());
    for (const auto& c : cands) {
        closest->push_back(c.p);
    }
    return !closest->empty();
}

IVox::PlaneHit IVox::EstimatePlane(const Vec3_t& query_world) const {
    PlaneHit hit;
    if (!query_world.allFinite()) {
        return hit;
    }

    const int k = std::max(options_.min_plane_points, options_.knn_max_num);
    std::vector<Vec3_t> neighbors;
    if (!GetClosestPoints(query_world, &neighbors, k, options_.knn_max_range)) {
        return hit;
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

    // Planarity: λ0 << λ1 (flat neighborhood). Reject edges/corners.
    const Vec3_t evals = solver.eigenvalues();
    const double e0 = std::max(0.0, evals(0));
    const double e1 = std::max(1e-12, evals(1));
    if (e0 / e1 > 0.1) {
        return PlaneHit{};
    }

    // Lightning esti_plane: reject if any neighbor is off-plane.
    if (options_.esti_plane_threshold > 0.0) {
        for (const auto& p : neighbors) {
            if (std::abs(hit.normal.dot(p) + hit.d) >
                options_.esti_plane_threshold) {
                return PlaneHit{};
            }
        }
    }
    hit.ok = true;
    return hit;
}

}  // namespace mapping
}  // namespace autonomy::localization::atlas
