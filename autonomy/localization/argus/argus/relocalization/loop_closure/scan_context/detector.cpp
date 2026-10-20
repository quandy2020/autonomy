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

#include "argus/relocalization/loop_closure/scan_context/detector.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

#include "argus/common/math/transform/transform.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace loop_closure {

namespace sc = relocalization::sc;

relocalization::sc::GridConfig ScanContextDetector::Grid() const {
    sc::GridConfig g;
    g.num_rings = config_.num_rings;
    g.num_sectors = config_.num_sectors;
    g.max_radius_m = config_.max_radius_m;
    return g;
}

ScanContextDetector::ScanContextDetector(ScanContextConfig config)
    : config_(std::move(config)) {}

int ScanContextDetector::database_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(keyframes_.size());
}

void ScanContextDetector::IndexKeyFrame(const KeyFrame& kf) {
    const sc::Descriptor descriptor = sc::BuildDescriptor(kf.cloud, Grid());
    const Eigen::VectorXd ring_key = sc::BuildRingKey(descriptor);
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.push_back(kf);
    descriptors_.push_back(descriptor);
    ring_keys_.push_back(ring_key);
}

std::vector<LoopCandidate> ScanContextDetector::Detect(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(mutex_);
    {
        const sc::Descriptor descriptor = sc::BuildDescriptor(kf.cloud, Grid());
        const Eigen::VectorXd ring_key = sc::BuildRingKey(descriptor);
        keyframes_.push_back(kf);
        descriptors_.push_back(descriptor);
        ring_keys_.push_back(ring_key);
    }

    const int database_size = static_cast<int>(keyframes_.size());
    const int query_index = database_size - 1;
    if (database_size <= config_.exclude_recent_count + 1) {
        return {};
    }

    const sc::GridConfig grid = Grid();
    const int lat_max =
        config_.enable_lateral_search ? std::max(0, config_.lateral_max_shift)
                                      : 0;
    const double lat_step = std::max(0.1, config_.lateral_step_m);
    const double sector_res =
        2.0 * M_PI / static_cast<double>(std::max(1, config_.num_sectors));

    struct Scored {
        int match_index = 0;
        double score = 0.0;
        int sector_shift = 0;
        double lateral_m = 0.0;
    };
    std::vector<Scored> scored;

    const int last_excluded =
        database_size - 1 - config_.exclude_recent_count;

    for (int lat = -lat_max; lat <= lat_max; ++lat) {
        const double dy = static_cast<double>(lat) * lat_step;
        const sensor::PointCloudXYZI q_cloud =
            sc::TranslateCloudY(kf.cloud, dy);
        const sc::Descriptor query_desc = sc::BuildDescriptor(q_cloud, grid);
        const Eigen::VectorXd query_ring = sc::BuildRingKey(query_desc);

        struct RingKeyCandidate {
            int index = 0;
            double ring_distance = 0;
        };
        std::vector<RingKeyCandidate> ranked;
        ranked.reserve(static_cast<std::size_t>(std::max(0, last_excluded)));
        for (int i = 0; i < last_excluded; ++i) {
            RingKeyCandidate candidate;
            candidate.index = i;
            candidate.ring_distance =
                (ring_keys_[static_cast<std::size_t>(i)] - query_ring).norm();
            ranked.push_back(candidate);
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const RingKeyCandidate& a, const RingKeyCandidate& b) {
                      return a.ring_distance < b.ring_distance;
                  });

        const int prefilter_count = std::min(
            config_.max_candidates * 3, static_cast<int>(ranked.size()));
        for (int k = 0; k < prefilter_count; ++k) {
            int shift = 0;
            const int match_index =
                ranked[static_cast<std::size_t>(k)].index;
            const double distance = sc::SectorShiftDistance(
                query_desc,
                descriptors_[static_cast<std::size_t>(match_index)], &shift);
            const double score = 1.0 - distance;
            if (score < config_.min_score) {
                continue;
            }
            Scored s;
            s.match_index = match_index;
            s.score = score;
            s.sector_shift = shift;
            s.lateral_m = dy;
            scored.push_back(s);
        }
    }

    std::sort(scored.begin(), scored.end(),
              [](const Scored& a, const Scored& b) { return a.score > b.score; });

    std::unordered_map<int, Scored> best_by_match;
    for (const Scored& s : scored) {
        auto it = best_by_match.find(s.match_index);
        if (it == best_by_match.end() || s.score > it->second.score) {
            best_by_match[s.match_index] = s;
        }
    }
    std::vector<Scored> unique;
    unique.reserve(best_by_match.size());
    for (const auto& kv : best_by_match) {
        unique.push_back(kv.second);
    }
    std::sort(unique.begin(), unique.end(),
              [](const Scored& a, const Scored& b) { return a.score > b.score; });

    std::vector<LoopCandidate> out;
    const int n_out =
        std::min(config_.max_candidates, static_cast<int>(unique.size()));
    out.reserve(static_cast<std::size_t>(n_out));
    for (int i = 0; i < n_out; ++i) {
        const Scored& s = unique[static_cast<std::size_t>(i)];
        LoopCandidate candidate;
        candidate.query_id =
            keyframes_[static_cast<std::size_t>(query_index)].id;
        candidate.match_id =
            keyframes_[static_cast<std::size_t>(s.match_index)].id;
        candidate.score = s.score;
        // Relative measurement from SC++ only (yaw + lateral). Do NOT use
        // \(T_{wq}^{-1}T_{wm}\) — that embeds odometry drift and makes the
        // loop factor nearly tautological / harmful after inject.
        const double yaw_delta =
            static_cast<double>(s.sector_shift) * sector_res;
        // Query was y-shifted by +lateral for matching → match in query ≈
        // FromXyzRpy(0, -lateral, 0, 0, 0, yaw).
        candidate.T_query_match = transform::FromXyzRpy(
            0.0, -s.lateral_m, 0.0, 0.0, 0.0, yaw_delta);
        out.push_back(candidate);
    }
    return out;
}

}  // namespace loop_closure
}  // namespace argus
