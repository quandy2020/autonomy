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
 * @file types.hpp
 * @brief Keyframe snapshot types and abstract loop detector interface.
 */

#ifndef ARGUS_RELOCALIZATION_LOOP_TYPES_HPP_
#define ARGUS_RELOCALIZATION_LOOP_TYPES_HPP_

#include <string>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace loop_closure {

/** @brief Keyframe snapshot for async loop detection (copied off ESIKF thread).
 */
struct KeyFrame {
    int id = 0;
    double stamp_sec = 0.0;
    /** @brief Body pose in world: \(p_w = R p_b + t\). */
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
    /** @brief Downsampled body-frame cloud for Scan Context. */
    sensor::PointCloudXYZI cloud;
    /** @brief Owning session; empty means live / current session. */
    std::string session_id;
    /** @brief Local id within `session_id` (for cross-session linking). */
    int session_local_id = -1;
};

/** @brief One geometric loop hypothesis from a detector. */
struct LoopCandidate {
    int query_id = 0;  /**< @brief Query keyframe id. */
    int match_id = 0;  /**< @brief Matched historical keyframe id. */
    double score = 0.0; /**< @brief Similarity in [0, 1]. */
    /** @brief Relative \(T_{\mathrm{query}\leftarrow\mathrm{match}}\)
     * (match → query). */
    transform::Rigid3d T_query_match = transform::Rigid3d::Identity();
    /** @brief ICP mean NN distance after refine (m); set by geometric verify. */
    double fitness_m = 1e9;
    /** @brief ICP inlier ratio after refine. */
    double inlier_ratio = 0.0;
};

/**
 * @brief World-frame correction after PoseGraph::Optimize for ESIKF inject.
 *
 * Apply as \(T' = \Delta_w\, T\) on the estimate thread only.
 */
struct LoopCorrection {
    int query_id = 0;
    int match_id = 0;
    transform::Rigid3d delta_w = transform::Rigid3d::Identity();
    double confidence = 0.0;
    /** @brief When false, estimate thread skips applying \(\Delta_w\) to ESIKF. */
    bool apply_to_state = true;
    /**
     * @brief When true, estimate thread clears synthetic eval drift \(T_d\).
     *        Only set for verified recovery (revisit signature / safe inject).
     */
    bool clear_eval_drift = false;
};

/** @brief Pluggable loop detector (Scan Context, etc.). */
class LoopDetectorBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(LoopDetectorBase);

    virtual ~LoopDetectorBase() = default;
    /**
     * @brief Index `kf` and return Top-K historical candidates (may be empty).
     * @param kf Query keyframe (pose + downsampled cloud).
     * @return Ranked loop candidates; empty when none pass internal gates.
     */
    virtual std::vector<LoopCandidate> Detect(const KeyFrame& kf) = 0;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_LOOP_TYPES_HPP_
