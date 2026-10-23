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

/**
 * @file map_self_cleaning.hpp
 * @brief Low-frequency voxel confidence update + prune (off ESIKF).
 */

#ifndef ARGUS_MAP_MAP_SELF_CLEANING_HPP_
#define ARGUS_MAP_MAP_SELF_CLEANING_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "argus/map/map_self_cleaning_config.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/perception/degeneracy_detector.hpp"

namespace argus {
namespace map {

/**
 * @brief Confidence bump (after map write) + async prune.
 *
 * - `UpdateConfidence`: call on estimate thread after `UpdateVoxelMap` (cheap).
 * - `Prune` / async worker: unique write lock; never inside ESIKF iteration.
 */
class MapSelfCleaning
{
public:
    /**
     * @brief Construct cleaner; call Start() to enable async prune.
     * @param config Thresholds, interval, and confidence update weights.
     */
    explicit MapSelfCleaning(MapSelfCleaningConfig config = {});
    ~MapSelfCleaning();

    MapSelfCleaning(const MapSelfCleaning&) = delete;
    MapSelfCleaning& operator=(const MapSelfCleaning&) = delete;

    /** @brief Start background prune worker. */
    void Start();
    /** @brief Stop worker and join thread. */
    void Stop();

    /**
     * @brief Bump confidence on `last_touched_voxels` (shared lock).
     * @param map Voxel map written in the last ESIKF cycle.
     * @param degen Degeneracy result used to scale confidence penalty.
     */
    void UpdateConfidence(VoxelMapManager& map,
                          const perception::DegeneracyResult& degen);

    /**
     * @brief Remove weak root voxels (unique lock via EraseVoxels).
     * @param map Map to prune in place.
     * @param threshold Minimum confidence to retain a voxel.
     * @return Number of root voxels erased.
     */
    int Prune(VoxelMapManager& map, float threshold);

    /**
     * @brief Non-blocking: schedule prune if `interval_sec` elapsed.
     *        Safe from ESIKF thread after map update.
     * @param map Target map for the next async prune (not owned).
     * @param now_sec Current time (s) for interval gating.
     */
    void MaybeRequestPrune(VoxelMapManager* map, double now_sec);

    /** @return Voxels removed in the last completed async prune. */
    int last_prune_count() const {
        return last_prune_count_.load();
    }

private:
    void WorkerLoop();

    MapSelfCleaningConfig config_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    std::mutex req_mutex_;
    std::condition_variable req_cv_;
    VoxelMapManager* pending_map_ = nullptr;
    bool prune_requested_ = false;
    double last_prune_sec_ = -1.0;
    std::atomic<int> last_prune_count_{0};
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_SELF_CLEANING_HPP_
