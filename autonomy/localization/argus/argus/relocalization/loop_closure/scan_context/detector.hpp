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
 * @file detector.hpp
 * @brief Scan Context / SC++ loop detector (Kim & Kim, IROS 2018+).
 */

#ifndef ARGUS_RELOCALIZATION_LOOP_SCAN_CONTEXT_DETECTOR_HPP_
#define ARGUS_RELOCALIZATION_LOOP_SCAN_CONTEXT_DETECTOR_HPP_

#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/relocalization/loop_closure/types.hpp"
#include "argus/relocalization/scan_context/codec.hpp"

namespace argus {
namespace loop_closure {

/** @brief Scan Context encoding and retrieval parameters. */
struct ScanContextConfig {
    int num_rings = 20;           /**< @brief SC rings. */
    int num_sectors = 60;         /**< @brief SC sectors. */
    double max_radius_m = 80.0;   /**< @brief Polar max range (m). */
    int max_candidates = 10;      /**< @brief Top-K after scoring. */
    int exclude_recent_count =
        30; /**< @brief Skip the newest N keyframes as matches. */
    double min_score = 0.5;       /**< @brief Min cosine similarity. */
    bool enable_lateral_search =
        true; /**< @brief SC++ query y-shift search. */
    int lateral_max_shift = 2;    /**< @brief Max |lateral| steps. */
    double lateral_step_m = 1.5;  /**< @brief Lateral step (m). */
};

/**
 * @brief Ring–sector Scan Context loop detector.
 * @note Detect / Index run on the loop-closure worker — never ESIKF.
 */
class ScanContextDetector : public LoopDetectorBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(ScanContextDetector);

    /**
     * @brief Construct with encoding and retrieval settings.
     * @param config Grid, thresholds, and SC++ lateral options.
     */
    explicit ScanContextDetector(ScanContextConfig config = {});

    /**
     * @brief Index `kf` and return Top-K historical loop candidates.
     * @param kf Query keyframe (pose + body cloud).
     * @return Ranked candidates (may be empty).
     */
    std::vector<LoopCandidate> Detect(const KeyFrame& kf) override;

    /**
     * @brief Index a keyframe without retrieval (e.g. prior session).
     * @param kf Keyframe to store.
     */
    void IndexKeyFrame(const KeyFrame& kf);

    /** @return Number of indexed keyframes. */
    int database_size() const;

private:
    relocalization::sc::GridConfig Grid() const;

    ScanContextConfig config_;
    mutable std::mutex mutex_;
    std::vector<KeyFrame> keyframes_;
    std::vector<relocalization::sc::Descriptor> descriptors_;
    std::vector<Eigen::VectorXd> ring_keys_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_LOOP_SCAN_CONTEXT_DETECTOR_HPP_
