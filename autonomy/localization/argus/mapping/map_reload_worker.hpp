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
 * @file map_reload_worker.hpp
 * @brief LTA-style historical cloud queue (transform off ESIKF; drain on it).
 */

#ifndef ARGUS_MAP_MAP_RELOAD_WORKER_HPP_
#define ARGUS_MAP_MAP_RELOAD_WORKER_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace map {

/** @brief Hint after accepted loop / reloc (world-frame Δ). */
struct MapReloadHint {
    int query_id = -1;
    int match_id = -1;
    transform::Rigid3d delta_w = transform::Rigid3d::Identity();
    double confidence = 0.0;
    double stamp_sec = 0.0;
};

/** @brief Settings for progressive history injection (LTAOM PointToAddHistorical). */
struct MapReloadConfig {
    bool enable = false;
    /** @brief Max points drained per estimate-thread call. */
    std::size_t points_per_frame = 20000;
    /** @brief Soft cap on queued clouds (drop oldest). */
    std::size_t max_queued_clouds = 32;
    /** @brief Voxel-ish stride when enqueueing (|dx|+|dy|+|dz| grid). */
    double downsample_resolution_m = 0.75;
    /**
     * @brief ±N prior-session keyframes around a cross-session match to reload.
     *        0 = only the matched keyframe.
     */
    int prior_neighbor_keyframes = 2;
};

/**
 * @brief Bounded queue of world-frame clouds to merge into the live map.
 *
 * Submit/Enqueue from loop worker; Drain only from the estimate thread.
 */
class MapReloadWorker
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(MapReloadWorker);

    explicit MapReloadWorker(MapReloadConfig config = {});

    void set_config(MapReloadConfig config);

    const MapReloadConfig& config() const {
        return config_;
    }

    /** @brief Record a correction hint (diagnostics / future chunk policy). */
    void SubmitHint(const MapReloadHint& hint);

    /**
     * @brief Enqueue a world cloud already corrected into the global frame.
     * @param cloud_w Points in world frame after LTA correction.
     */
    void EnqueueCorrectedCloud(sensor::PointCloudXYZI cloud_w);

    /**
     * @brief Apply \(p' = \Delta_w p\) and enqueue (body/world cloud as given).
     * @param cloud Input points.
     * @param delta_w World correction \(T' = \Delta_w T\).
     */
    void EnqueueTransformed(const sensor::PointCloudXYZI& cloud,
                            const transform::Rigid3d& delta_w);

    /**
     * @brief Pop up to `budget` points for live-map insertion.
     * @param[out] out Appended points (not cleared).
     * @param budget Max points this call.
     * @return Points appended.
     */
    std::size_t Drain(sensor::PointCloudXYZI* out, std::size_t budget);

    /** @return Queued cloud count (approx). */
    std::size_t queue_size() const;

    std::vector<MapReloadHint> TakeHints();

private:
    sensor::PointCloudXYZI Downsample(const sensor::PointCloudXYZI& in) const;

    /** @brief Queued cloud with consume offset (avoid O(n) erase on partial drain). */
    struct QueuedCloud {
        sensor::PointCloudXYZI cloud;
        std::size_t offset = 0;
    };

    MapReloadConfig config_;
    mutable std::mutex mutex_;
    std::deque<QueuedCloud> queue_;
    std::vector<MapReloadHint> hints_;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_RELOAD_WORKER_HPP_
