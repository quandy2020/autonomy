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

#include "argus/relocalization/scan_context/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include "argus/common/math/transform/transform.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace relocalization {

namespace {

int64_t GridKey(int ix, int iy) {
    return (static_cast<int64_t>(static_cast<uint32_t>(ix)) << 32) |
           static_cast<uint32_t>(iy);
}

}  // namespace

sc::GridConfig ScanContextPlaceIndex::Grid() const {
    sc::GridConfig g;
    g.num_rings = config_.num_rings;
    g.num_sectors = config_.num_sectors;
    g.max_radius_m = config_.max_radius_m;
    return g;
}

ScanContextPlaceIndex::ScanContextPlaceIndex(PlaceRecognitionConfig config)
    : config_(std::move(config)) {}

void ScanContextPlaceIndex::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
}

int ScanContextPlaceIndex::database_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(entries_.size());
}

void ScanContextPlaceIndex::IndexKeyFrame(
    int id, const transform::Rigid3d& T_w_b,
    const sensor::PointCloudXYZI& cloud_b) {
    if (cloud_b.size() <
        static_cast<std::size_t>(std::max(1, config_.min_points_per_keyframe))) {
        return;
    }
    const sc::Descriptor descriptor = sc::BuildDescriptor(cloud_b, Grid());
    if (descriptor.norm() < 1e-9) {
        return;
    }
    Entry entry;
    entry.id = id;
    entry.translation = T_w_b.translation();
    entry.descriptor = descriptor;
    entry.ring_key = sc::BuildRingKey(descriptor);
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.push_back(std::move(entry));
}

void ScanContextPlaceIndex::BuildFromWorldCloud(
    const sensor::PointCloudXYZI& cloud_w) {
    Clear();
    if (cloud_w.empty()) {
        return;
    }

    const double spacing = std::max(1.0, config_.keyframe_spacing_m);
    const double radius = config_.max_radius_m;
    const int ring_cells =
        std::max(1, static_cast<int>(std::ceil(radius / spacing)));

    struct Cell {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
        int ix = 0;
        int iy = 0;
        std::vector<std::size_t> indices;
    };
    std::unordered_map<int64_t, Cell> cells;
    cells.reserve(cloud_w.size() / 8 + 1);

    for (std::size_t i = 0; i < cloud_w.size(); ++i) {
        const double x = cloud_w[i].x;
        const double y = cloud_w[i].y;
        const double z = cloud_w[i].z;
        const int ix = static_cast<int>(std::floor(x / spacing));
        const int iy = static_cast<int>(std::floor(y / spacing));
        Cell& cell = cells[GridKey(ix, iy)];
        if (cell.count == 0) {
            cell.ix = ix;
            cell.iy = iy;
        }
        cell.sum += Eigen::Vector3d(x, y, z);
        ++cell.count;
        cell.indices.push_back(i);
    }

    int next_id = 0;
    for (const auto& kv : cells) {
        const Cell& cell = kv.second;
        if (cell.count < config_.min_points_per_keyframe) {
            continue;
        }
        // Lattice sensor at XY cell origin (z = 0); keep absolute z in local
        // cloud for max-height encoding.
        const Eigen::Vector3d sensor(cell.ix * spacing, cell.iy * spacing, 0.0);

        sensor::PointCloudXYZI local;
        local.reserve(static_cast<std::size_t>(cell.count) * 4);
        for (int dx = -ring_cells; dx <= ring_cells; ++dx) {
            for (int dy = -ring_cells; dy <= ring_cells; ++dy) {
                const auto it = cells.find(GridKey(cell.ix + dx, cell.iy + dy));
                if (it == cells.end()) {
                    continue;
                }
                for (std::size_t idx : it->second.indices) {
                    const double lx = cloud_w[idx].x - sensor.x();
                    const double ly = cloud_w[idx].y - sensor.y();
                    if (std::hypot(lx, ly) > radius) {
                        continue;
                    }
                    sensor::PointType p = cloud_w[idx];
                    p.x = static_cast<float>(lx);
                    p.y = static_cast<float>(ly);
                    p.z = cloud_w[idx].z;
                    local.push_back(p);
                }
            }
        }
        if (local.size() <
            static_cast<std::size_t>(config_.min_points_per_keyframe)) {
            continue;
        }

        IndexKeyFrame(next_id++, transform::Rigid3d::Translation(sensor),
                      local);
    }
}

std::vector<PlaceCandidate> ScanContextPlaceIndex::Retrieve(
    const sensor::PointCloudXYZI& query_cloud_b) const {
    std::vector<PlaceCandidate> out;
    if (query_cloud_b.empty()) {
        return out;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.empty()) {
        return out;
    }

    const sc::GridConfig grid = Grid();
    const int lat_max =
        config_.enable_lateral_search ? std::max(0, config_.lateral_max_shift)
                                      : 0;
    const double lat_step = std::max(0.1, config_.lateral_step_m);
    const double sector_res =
        2.0 * M_PI / static_cast<double>(std::max(1, config_.num_sectors));

    struct Scored {
        int entry_index = 0;
        double score = 0.0;
        int sector_shift = 0;
        double lateral_m = 0.0;
    };
    std::vector<Scored> scored;
    scored.reserve(static_cast<std::size_t>(config_.top_k) * 2);

    for (int lat = -lat_max; lat <= lat_max; ++lat) {
        const double dy = static_cast<double>(lat) * lat_step;
        const sensor::PointCloudXYZI q =
            sc::TranslateCloudY(query_cloud_b, dy);
        const sc::Descriptor query_desc = sc::BuildDescriptor(q, grid);
        if (query_desc.norm() < 1e-9) {
            continue;
        }
        const Eigen::VectorXd query_ring = sc::BuildRingKey(query_desc);

        struct RingCand {
            int index = 0;
            double ring_distance = 0.0;
        };
        std::vector<RingCand> ranked;
        ranked.reserve(entries_.size());
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            RingCand c;
            c.index = static_cast<int>(i);
            c.ring_distance = (entries_[i].ring_key - query_ring).norm();
            ranked.push_back(c);
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const RingCand& a, const RingCand& b) {
                      return a.ring_distance < b.ring_distance;
                  });

        const int prefilter = std::min(
            config_.ring_key_prefilter_k, static_cast<int>(ranked.size()));
        for (int k = 0; k < prefilter; ++k) {
            const int ei = ranked[static_cast<std::size_t>(k)].index;
            int shift = 0;
            const double distance = sc::SectorShiftDistance(
                query_desc, entries_[static_cast<std::size_t>(ei)].descriptor,
                &shift);
            const double score = 1.0 - distance;
            if (score < config_.min_score) {
                continue;
            }
            Scored s;
            s.entry_index = ei;
            s.score = score;
            s.sector_shift = shift;
            s.lateral_m = dy;
            scored.push_back(s);
        }
    }

    std::sort(scored.begin(), scored.end(),
              [](const Scored& a, const Scored& b) { return a.score > b.score; });

    std::unordered_map<int, Scored> best_by_id;
    for (const Scored& s : scored) {
        const int id = entries_[static_cast<std::size_t>(s.entry_index)].id;
        auto it = best_by_id.find(id);
        if (it == best_by_id.end() || s.score > it->second.score) {
            best_by_id[id] = s;
        }
    }
    std::vector<Scored> unique;
    unique.reserve(best_by_id.size());
    for (const auto& kv : best_by_id) {
        unique.push_back(kv.second);
    }
    std::sort(unique.begin(), unique.end(),
              [](const Scored& a, const Scored& b) { return a.score > b.score; });

    const int n_out =
        std::min(config_.top_k, static_cast<int>(unique.size()));
    out.reserve(static_cast<std::size_t>(n_out));
    for (int i = 0; i < n_out; ++i) {
        const Scored& s = unique[static_cast<std::size_t>(i)];
        const Entry& e = entries_[static_cast<std::size_t>(s.entry_index)];
        PlaceCandidate cand;
        cand.keyframe_id = e.id;
        cand.score = s.score;
        cand.yaw_shift_rad = static_cast<double>(s.sector_shift) * sector_res;
        cand.lateral_shift_m = s.lateral_m;
        const Eigen::Vector3d t =
            e.translation + Eigen::Vector3d(0.0, -s.lateral_m, 0.0);
        cand.T_w_b =
            transform::FromXyzRpy(t.x(), t.y(), t.z(), 0.0, 0.0, 0.0);
        out.push_back(cand);
    }
    return out;
}

}  // namespace relocalization
}  // namespace argus
