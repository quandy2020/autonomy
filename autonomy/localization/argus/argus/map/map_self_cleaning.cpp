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

#include "argus/map/map_self_cleaning.hpp"

#include <unordered_set>
#include <utility>
#include <vector>

#include "argus/map/map_confidence.hpp"

namespace argus {
namespace map {

MapSelfCleaning::MapSelfCleaning(MapSelfCleaningConfig config)
    : config_(std::move(config)) {
    config_.confidence_update.degeneracy_penalty = config_.degeneracy_penalty;
}

MapSelfCleaning::~MapSelfCleaning() {
    Stop();
}

void MapSelfCleaning::Start() {
    if (!config_.enable) {
        return;
    }
    if (running_.exchange(true)) {
        return;
    }
    worker_ = std::thread([this] { WorkerLoop(); });
}

void MapSelfCleaning::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(req_mutex_);
        prune_requested_ = true;
        pending_map_ = nullptr;
    }
    req_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void MapSelfCleaning::UpdateConfidence(
    VoxelMapManager& map, const perception::DegeneracyResult& degen) {
    if (!config_.enable) {
        return;
    }
    map.SetUpdateUnderDegeneracy(degen.is_degenerated);
    std::shared_lock<std::shared_mutex> lock(map.map_mutex());
    // Deduplicate locations: one bump per root voxel per call.
    std::unordered_set<int64_t> seen;
    seen.reserve(map.last_touched_voxels().size());
    for (const VoxelLocation& loc : map.last_touched_voxels()) {
        const int64_t key =
            (loc.x * 73856093) ^ (loc.y * 19349663) ^ (loc.z * 83492791);
        if (!seen.insert(key).second) {
            continue;
        }
        auto it = map.voxel_map_.find(loc);
        if (it == map.voxel_map_.end() || it->second == nullptr) {
            continue;
        }
        BumpMapConfidence(&it->second->confidence_, degen.is_degenerated,
                          config_.confidence_update);
    }
}

int MapSelfCleaning::Prune(VoxelMapManager& map, float threshold) {
    if (!config_.enable) {
        return 0;
    }
    std::vector<VoxelLocation> doomed;
    {
        std::shared_lock<std::shared_mutex> lock(map.map_mutex());
        doomed.reserve(map.voxel_map_.size() / 8 + 1);
        for (const auto& kv : map.voxel_map_) {
            if (kv.second == nullptr) {
                continue;
            }
            if (ShouldPruneVoxel(kv.second->confidence_, threshold,
                                 config_.min_observation_count_to_keep)) {
                doomed.push_back(kv.first);
            }
        }
    }
    const int n = map.EraseVoxels(doomed);
    last_prune_count_.store(n);
    return n;
}

void MapSelfCleaning::MaybeRequestPrune(VoxelMapManager* map, double now_sec) {
    if (!config_.enable || map == nullptr || !running_.load()) {
        return;
    }
    if (last_prune_sec_ >= 0.0 &&
        (now_sec - last_prune_sec_) < config_.interval_sec) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(req_mutex_);
        pending_map_ = map;
        prune_requested_ = true;
        last_prune_sec_ = now_sec;
    }
    req_cv_.notify_one();
}

void MapSelfCleaning::WorkerLoop() {
    while (running_.load()) {
        VoxelMapManager* map = nullptr;
        {
            std::unique_lock<std::mutex> lock(req_mutex_);
            req_cv_.wait(
                lock, [this] { return prune_requested_ || !running_.load(); });
            if (!running_.load()) {
                break;
            }
            map = pending_map_;
            pending_map_ = nullptr;
            prune_requested_ = false;
        }
        if (map != nullptr) {
            Prune(*map, config_.confidence_threshold);
        }
    }
}

}  // namespace map
}  // namespace argus
