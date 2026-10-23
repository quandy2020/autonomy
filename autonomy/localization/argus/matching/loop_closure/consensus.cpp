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

#include "argus/relocalization/loop_closure/consensus.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

#include "argus/common/math/transform/transform.hpp"

namespace argus {
namespace loop_closure {
namespace {

double YawOf(const transform::Rigid3d& T) {
    return transform::GetYaw(T);
}

double PoseConsistency(const LoopCandidate& cand,
                       const MultiModalConsensusConfig& config) {
    const double yaw = std::abs(YawOf(cand.T_query_match));
    const double trans = cand.T_query_match.translation().norm();
    const double yaw_score =
        1.0 - std::clamp(yaw / std::max(config.max_relative_yaw_rad, 1e-3),
                         0.0, 1.0);
    const double trans_score =
        1.0 -
        std::clamp(trans / std::max(config.max_relative_trans_m, 1e-3), 0.0,
                   1.0);
    return 0.5 * yaw_score + 0.5 * trans_score;
}

}  // namespace

MultiModalConsensus::MultiModalConsensus(MultiModalConsensusConfig config)
    : config_(std::move(config)) {}

bool MultiModalConsensus::AcceptCandidate(const LoopCandidate& cand,
                                          double path_separation_m,
                                          double* confidence,
                                          double visual_score) const {
    if (std::abs(cand.query_id - cand.match_id) < config_.min_id_gap) {
        if (confidence != nullptr) {
            *confidence = 0.0;
        }
        return false;
    }
    // Hard path gate (soft topology score alone was too weak on Spires).
    if (path_separation_m < config_.min_path_separation_m) {
        if (confidence != nullptr) {
            *confidence = 0.0;
        }
        return false;
    }

    const double topology_score = std::min(
        1.0,
        path_separation_m / std::max(config_.min_path_separation_m, 1e-3));
    const double visual_score_clamped =
        visual_score >= 0.0 ? std::clamp(visual_score, 0.0, 1.0) : 0.5;
    const double geometric_score = std::clamp(cand.score, 0.0, 1.0);
    const double pose_score = PoseConsistency(cand, config_);

    const double weight_sum = config_.geometric_weight +
                              config_.visual_weight +
                              config_.topology_weight +
                              config_.pose_consistency_weight;
    const double inv_weight = 1.0 / std::max(weight_sum, 1e-9);
    const double fused =
        inv_weight *
        (config_.geometric_weight * geometric_score +
         config_.visual_weight * visual_score_clamped +
         config_.topology_weight * topology_score +
         config_.pose_consistency_weight * pose_score);
    if (confidence != nullptr) {
        *confidence = fused;
    }
    return fused >= config_.min_confidence;
}

}  // namespace loop_closure
}  // namespace argus
