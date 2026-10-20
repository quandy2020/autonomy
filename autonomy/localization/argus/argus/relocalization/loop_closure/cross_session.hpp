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
 * @file cross_session.hpp
 * @brief Build chain loop factors between map sessions; update anchors.
 */

#ifndef ARGUS_RELOCALIZATION_LOOP_CROSS_SESSION_HPP_
#define ARGUS_RELOCALIZATION_LOOP_CROSS_SESSION_HPP_

#include <string>
#include <vector>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/map/map_session.hpp"

namespace argus {
namespace loop_closure {

struct CrossSessionConfig {
    bool enable = false;
    double min_confidence = 0.65;
    double loop_information_scale = 150.0;
    /** @brief If true, freeze prior-session nodes (high info odom stubs). */
    bool freeze_prior_nodes = true;
};

struct CrossSessionLink {
    std::string session_prior;
    std::string session_current;
    int local_id_prior = 0;
    int local_id_current = 0;
    /**
     * @brief Measured relative \(T_{\mathrm{cur}\leftarrow\mathrm{prior}}\)
     *        (prior body → current body), same convention as LoopCandidate.
     */
    transform::Rigid3d T_cur_prior = transform::Rigid3d::Identity();
    double confidence = 0.0;
};

/**
 * @brief Inject prior trajectories into PoseGraph and emit chain loop factors.
 *
 * Call only from the async loop-closure worker (never ESIKF).
 */
class CrossSessionLinker
{
public:
    /**
     * @brief Wire linker to a session registry owned by the loop worker.
     * @param sessions Non-null MapSession for anchors and trajectories.
     * @param config Confidence and freeze options.
     */
    CrossSessionLinker(map::MapSession* sessions,
                       CrossSessionConfig config = {});

    /** @return Active cross-session configuration. */
    const CrossSessionConfig& config() const {
        return config_;
    }

    /**
     * @brief Add prior session nodes (world poses) into `graph`.
     * @param graph Pose graph to extend with prior-session nodes.
     * @param session_id Registered session identifier.
     * @return Number of nodes added (0 if missing / already present).
     */
    int AddSessionToGraph(backend::PoseGraph& graph,
                          const std::string& session_id);

    /**
     * @brief Build loop factors for one A↔B hit (may be empty if invalid).
     * @param link Cross-session relative measurement and session ids.
     * @return Chain loop factors; empty when link is rejected.
     */
    std::vector<backend::LoopFactor> BuildChainFactors(
        const CrossSessionLink& link) const;

    /**
     * @brief TryAcceptLoops (FPR) + gauge restore prior + refresh current
     *        `T_world_session`.
     * @param graph Pose graph to optimize in place.
     * @param link Cross-session link to apply.
     * @param fpr Hard FPR gates (same as intra-session).
     * @param max_iterations LM / GN iteration cap for Optimize.
     * @param[out] opt_out Optional TryAcceptLoops diagnostics.
     * @return false if confidence / sessions invalid or FPR rejects.
     */
    bool LinkAndOptimize(backend::PoseGraph& graph,
                         const CrossSessionLink& link,
                         const backend::PoseGraphFprConfig& fpr,
                         int max_iterations = 10,
                         backend::PoseGraphOptimizeResult* opt_out = nullptr);

private:
    map::MapSession* sessions_ = nullptr;
    CrossSessionConfig config_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_LOOP_CROSS_SESSION_HPP_
