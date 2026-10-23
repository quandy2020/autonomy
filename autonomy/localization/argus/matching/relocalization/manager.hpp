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
 * @file manager.hpp
 * @brief Async relocalization facade (pool-owned BnB; atomic result handoff).
 */

#ifndef ARGUS_RELOCALIZATION_MANAGER_HPP_
#define ARGUS_RELOCALIZATION_MANAGER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/relocalization/bbs/engine.hpp"
#include "argus/relocalization/candidate.hpp"
#include "argus/relocalization/prior_map.hpp"
#include "argus/relocalization/types.hpp"
#include "argus/relocalization/config.hpp"
#include "argus/relocalization/thread_pool.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

/** @brief Signals used to decide tracking_lost (pure, testable). */
struct TrackingLostSignals {
    bool enable_gating = false;
    bool lidar_valid = true;
    bool lidar_healthy = true;
    bool enable_degeneracy = false;
    bool is_degenerated = false;
    bool cloud_empty = false;
};

/** @return true if the estimate thread should count a lost frame. */
inline bool IsTrackingLost(const TrackingLostSignals& signals) {
    if (signals.cloud_empty) {
        return true;
    }
    if (!signals.lidar_healthy) {
        return true;
    }
    if (signals.enable_gating && !signals.lidar_valid) {
        return true;
    }
    if (signals.enable_degeneracy && signals.is_degenerated) {
        return true;
    }
    return false;
}

/**
 * @brief Owns the dedicated pool + search engine.
 *
 * - `RequestRelocalization`: non-blocking (ESIKF-safe).
 * - `TryAutoTrigger`: streak + cooldown for `tracking_lost` (still
 * non-blocking).
 * - `TakeResults`: drain accepted poses for atomic inject on the estimate
 * thread.
 * - Never runs BnB on the caller thread.
 */
class RelocalizationManager
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(RelocalizationManager);

    explicit RelocalizationManager(RelocalizationConfig config = {});
    ~RelocalizationManager();

    RelocalizationManager(const RelocalizationManager&) = delete;
    RelocalizationManager& operator=(const RelocalizationManager&) = delete;

    void Start();
    void Stop();

    void SetPriorMap(PriorOccupancyMap::ConstSharedPtr map);
    /**
     * @brief Set occupancy prior and prefer `cloud_w` for SC++ place index.
     * @param map Occupancy prior.
     * @param cloud_w Source world cloud (e.g. from LoadPathBundle).
     */
    void SetPriorMap(PriorOccupancyMap::ConstSharedPtr map,
                     const sensor::PointCloudXYZI& cloud_w);
    /** @brief Convenience: occupancy + world cloud from a loader bundle. */
    void SetPriorMap(const PriorMapBundle& bundle);
    /**
     * @brief Optional upstream place source (visual, …); LiDAR-gated by SC.
     * @param source Shared candidate source; nullptr clears.
     */
    void SetAuxCandidateSource(PlaceCandidateSource::SharedPtr source);
    bool HasPriorMap() const;
    /** @return SC++ place-index size (0 if disabled). */
    int place_index_size() const;

    /** @brief Non-blocking enqueue; returns false if disabled / pool reject. */
    bool RequestRelocalization(RelocRequest request);

    /**
     * @brief Auto-trigger when `trigger==tracking_lost` and streak/cooldown
     * allow.
     * @param lost  Current-frame tracking-lost flag (`IsTrackingLost`).
     * @param stamp_sec  Estimate-thread time for cooldown.
     * @param request  Query cloud (+ optional guess); id assigned if 0.
     * @return true if a task was enqueued.
     */
    bool TryAutoTrigger(bool lost, double stamp_sec, RelocRequest request);

    /** @brief Drain completed results (success or cancelled). */
    std::vector<RelocResult> TakeResults();

    void CancelPending();

    int pending_count() const;
    int success_count() const {
        return success_count_.load();
    }
    int dropped_count() const;
    int lost_streak() const {
        return lost_streak_;
    }
    int auto_trigger_count() const {
        return auto_trigger_count_;
    }

    RelocalizationThreadPool& pool() {
        return pool_;
    }
    const RelocalizationConfig& config() const {
        return config_;
    }

private:
    RelocalizationConfig config_;
    RelocalizationThreadPool pool_;
    BbsSearchEngine::SharedPtr engine_;
    mutable std::mutex engine_mutex_;

    std::mutex results_mutex_;
    std::vector<RelocResult> results_;
    std::atomic<uint64_t> next_id_{1};
    std::atomic<int> success_count_{0};

    int lost_streak_ = 0;
    double last_auto_request_sec_ = -1.0e30;
    int auto_trigger_count_ = 0;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_MANAGER_HPP_
