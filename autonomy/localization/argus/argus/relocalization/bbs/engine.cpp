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

#include "argus/relocalization/bbs/engine.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace argus {
namespace relocalization {

BbsSearchEngine::BbsSearchEngine(PriorOccupancyMap::ConstSharedPtr map,
                                 BranchAndBoundConfig branch_and_bound_config,
                                 PlaceRecognitionConfig place_recognition)
    : map_(std::move(map)),
      branch_and_bound_config_(std::move(branch_and_bound_config)),
      place_recognition_(std::move(place_recognition)),
      place_index_(place_recognition_) {
    RebuildPlaceIndexLocked();
}

void BbsSearchEngine::RebuildPlaceIndexLocked() {
    place_index_.set_config(place_recognition_);
    place_index_.Clear();
    if (!place_recognition_.enable || !map_) {
        return;
    }
    if (!preferred_index_cloud_.empty()) {
        place_index_.BuildFromWorldCloud(preferred_index_cloud_);
    } else {
        place_index_.BuildFromWorldCloud(map_->OccupiedCentersCloud());
    }
}

void BbsSearchEngine::SetMap(PriorOccupancyMap::ConstSharedPtr map) {
    map_ = std::move(map);
    preferred_index_cloud_.clear();
    RebuildPlaceIndexLocked();
}

void BbsSearchEngine::SetMap(PriorOccupancyMap::ConstSharedPtr map,
                             const sensor::PointCloudXYZI& cloud_w) {
    map_ = std::move(map);
    preferred_index_cloud_ = cloud_w;
    RebuildPlaceIndexLocked();
}

void BbsSearchEngine::BuildPlaceIndexFromCloud(
    const sensor::PointCloudXYZI& cloud_w) {
    preferred_index_cloud_ = cloud_w;
    place_index_.set_config(place_recognition_);
    if (!place_recognition_.enable) {
        place_index_.Clear();
        return;
    }
    place_index_.BuildFromWorldCloud(cloud_w);
}

void BbsSearchEngine::SetPlaceRecognitionConfig(
    PlaceRecognitionConfig config) {
    place_recognition_ = std::move(config);
    RebuildPlaceIndexLocked();
}

void BbsSearchEngine::SetAuxCandidateSource(
    PlaceCandidateSource::SharedPtr source) {
    aux_source_ = std::move(source);
}

int BbsSearchEngine::place_index_size() const {
    return place_index_.database_size();
}

std::vector<PlaceCandidate> BbsSearchEngine::GateWithScanContext(
    const sensor::PointCloudXYZI& query_cloud_b,
    const std::vector<PlaceCandidate>& seeds) const {
    if (seeds.empty()) {
        return {};
    }
    if (place_index_.database_size() <= 0) {
        // No LiDAR DB yet — pass seeds through (BnB is the geometric check).
        return seeds;
    }
    const std::vector<PlaceCandidate> sc = place_index_.Retrieve(query_cloud_b);
    if (sc.empty()) {
        return {};
    }
    // Keep aux seeds whose XY is near a SC-accepted place (LiDAR gate).
    constexpr double kGateRadiusM = 15.0;
    std::vector<PlaceCandidate> gated;
    gated.reserve(seeds.size());
    for (const PlaceCandidate& seed : seeds) {
        bool near = false;
        double best_sc = 0.0;
        PlaceCandidate best = seed;
        for (const PlaceCandidate& s : sc) {
            const double d =
                (seed.T_w_b.translation() - s.T_w_b.translation())
                    .head<2>()
                    .norm();
            if (d <= kGateRadiusM && s.score >= best_sc) {
                near = true;
                best_sc = s.score;
                best = seed;
                best.score = s.score;
                // Prefer SC yaw-free lattice translation when closer.
                best.T_w_b = s.T_w_b;
            }
        }
        if (near) {
            gated.push_back(best);
        }
    }
    std::sort(gated.begin(), gated.end(),
              [](const PlaceCandidate& a, const PlaceCandidate& b) {
                  return a.score > b.score;
              });
    if (static_cast<int>(gated.size()) > place_recognition_.top_k) {
        gated.resize(static_cast<std::size_t>(place_recognition_.top_k));
    }
    return gated;
}

std::vector<PlaceCandidate> BbsSearchEngine::CollectCandidates(
    const sensor::PointCloudXYZI& query_cloud_b) const {
    if (aux_source_ && aux_source_->database_size() > 0) {
        const std::vector<PlaceCandidate> aux =
            aux_source_->Retrieve(query_cloud_b);
        const std::vector<PlaceCandidate> gated =
            GateWithScanContext(query_cloud_b, aux);
        if (!gated.empty()) {
            return gated;
        }
        // Aux empty after gate → fall through to pure SC.
    }
    return place_index_.Retrieve(query_cloud_b);
}

RelocResult BbsSearchEngine::SearchFull(
    const RelocRequest& request, std::atomic<bool>* cancel,
    const BranchAndBoundConfig& bnb_config) const {
    BranchAndBound bnb(map_, bnb_config);
    return bnb.Search(request, cancel);
}

RelocResult BbsSearchEngine::SearchWithPlaceCrop(
    const RelocRequest& request, std::atomic<bool>* cancel) const {
    const std::vector<PlaceCandidate> candidates =
        CollectCandidates(request.query_cloud);

    RelocResult best;
    best.request_id = request.id;
    best.best.score = -1.0;

    BranchAndBoundConfig cropped = branch_and_bound_config_;
    // Lattice keyframes need ±~keyframe_spacing around each seed.
    const int spacing_steps = static_cast<int>(std::ceil(
        place_recognition_.keyframe_spacing_m /
        std::max(1e-3, branch_and_bound_config_.resolution)));
    cropped.search_radius_steps =
        std::max({1, place_recognition_.crop_radius_steps, spacing_steps});

    for (const PlaceCandidate& cand : candidates) {
        if (cancel != nullptr && cancel->load()) {
            best.cancelled = true;
            return best;
        }
        RelocRequest seeded = request;
        seeded.has_initial_guess = true;
        seeded.initial_guess = cand.T_w_b;
        RelocResult r = SearchFull(seeded, cancel, cropped);
        if (r.cancelled) {
            return r;
        }
        if (r.best.score > best.best.score) {
            best = r;
        }
        if (best.success) {
            return best;
        }
    }

    if (!best.success && place_recognition_.fallback_full_search) {
        RelocResult full =
            SearchFull(request, cancel, branch_and_bound_config_);
        if (full.cancelled) {
            return full;
        }
        if (full.best.score > best.best.score) {
            return full;
        }
    }
    return best;
}

RelocResult BbsSearchEngine::Search(const RelocRequest& request,
                                    std::atomic<bool>* cancel) {
    if (!map_) {
        RelocResult empty;
        empty.request_id = request.id;
        return empty;
    }
    const bool use_crop =
        place_recognition_.enable &&
        (place_index_.database_size() > 0 ||
         (aux_source_ && aux_source_->database_size() > 0));
    if (use_crop) {
        return SearchWithPlaceCrop(request, cancel);
    }
    return SearchFull(request, cancel, branch_and_bound_config_);
}

}  // namespace relocalization
}  // namespace argus
