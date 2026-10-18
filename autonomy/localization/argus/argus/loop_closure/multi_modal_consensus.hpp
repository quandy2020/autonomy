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
 * @file multi_modal_consensus.hpp
 * @brief Fuse geometric / visual / topology scores for loop-candidate gating.
 */

#ifndef ARGUS_LOOP_CLOSURE_MULTI_MODAL_CONSENSUS_HPP_
#define ARGUS_LOOP_CLOSURE_MULTI_MODAL_CONSENSUS_HPP_

#include "argus/loop_closure/loop_detector_base.hpp"

namespace argus {
namespace loop_closure {

/** @brief Weights and rejection thresholds for multi-modal loop arbitration. */
struct MultiModalConsensusConfig {
    double geometric_weight =
        0.45; /**< @brief Scan-context / geometric score weight. */
    double visual_weight = 0.25; /**< @brief Visual descriptor score weight. */
    double topology_weight = 0.20; /**< @brief Path / topology score weight. */
    /** @brief Yaw / translation consistency of
     * \(T_{\mathrm{query}\leftarrow\mathrm{match}}\). */
    double pose_consistency_weight = 0.10;
    double min_confidence =
        0.65; /**< @brief Accept if fused confidence ≥ this. */
    /** @brief Reject if |id_query - id_match| below this (temporal proximity).
     */
    int min_id_gap = 30;
    /** @brief Prefer candidates whose odometry path length exceeds this (m). */
    double min_path_separation_m = 10.0;
    /** @brief Soft-penalize relative yaw beyond this (rad) in pose consistency.
     */
    double max_relative_yaw_rad = 0.6;
    /** @brief Soft-penalize relative translation beyond this (m). */
    double max_relative_trans_m = 15.0;
};

/**
 * @brief Score fusion and hard gates for loop-closure candidates.
 */
class MultiModalConsensus
{
public:
    /**
     * @brief Construct with fusion weights / gates.
     * @param config Consensus parameters.
     */
    explicit MultiModalConsensus(MultiModalConsensusConfig config = {});

    /**
     * @brief Fuse scores and hard-gate a loop candidate.
     * @param cand Loop candidate (ids, relative pose, geometric score).
     * @param path_separation_m Approximate travel distance between match and
     * query.
     * @param[out] confidence Fused confidence in [0,1] when non-null.
     * @param visual_score Optional [0,1] visual descriptor score; &lt;0 →
     * neutral 0.5.
     * @return True if the candidate passes gates and min confidence.
     */
    bool AcceptCandidate(const LoopCandidate& cand, double path_separation_m,
                         double* confidence,
                         double visual_score = -1.0) const;

private:
    MultiModalConsensusConfig config_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_LOOP_CLOSURE_MULTI_MODAL_CONSENSUS_HPP_
