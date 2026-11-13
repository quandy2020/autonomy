/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file pipeline.cpp
 * @brief Run L2–L6 on an EpisodeBundle into EpisodeSidecar.
 */

#include "autonomy/system/recorder/pipeline.hpp"

#include "autonomy/system/recorder/bt_attribution.hpp"
#include "autonomy/system/recorder/event_graph.hpp"
#include "autonomy/system/recorder/hash_chain.hpp"
#include "autonomy/system/recorder/metrics.hpp"
#include "autonomy/system/recorder/observation.hpp"
#include "autonomy/system/recorder/replay.hpp"
#include "autonomy/system/recorder/root_cause.hpp"
#include "autonomy/system/recorder/version_tuple.hpp"

namespace autonomy {
namespace system {
namespace recorder {

proto::EpisodeSidecar RunEvidencePipeline(const EpisodeBundle& bundle,
                                          double match_threshold,
                                          double coverage_floor) {
    proto::EpisodeSidecar sidecar;
    *sidecar.mutable_header() = bundle.header;
    sidecar.mutable_header()->set_admitted(IsAdmitted(bundle.header.version()));
    for (const auto& t : bundle.triggers) {
        *sidecar.add_triggers() = t;
    }
    *sidecar.mutable_graph() =
        BuildEventGraph(bundle.transitions, bundle.triggers);
    for (const auto& obs : bundle.observations) {
        if (AcceptObservation(obs)) {
            *sidecar.add_observations() = obs;
        }
    }
    for (auto& card : RankRootCauses(sidecar.graph())) {
        if (bundle.reroll) {
            RerollHypothesis(bundle.reroll, bundle.fail_set, bundle.success_set,
                             bundle.regression_limit, &card);
        }
        *sidecar.add_root_causes() = card;
    }
    if (auto fail = AttributeBtFailure(bundle.transitions)) {
        *sidecar.mutable_bt_failure() = *fail;
    }

    SafetyHashChain chain(bundle.header.robot_id());
    bool safety_ok = true;
    for (const auto& ev : bundle.safety_events) {
        *sidecar.add_safety_events() = chain.Append(ev);
    }
    std::vector<proto::SafetyEvent> sealed(
        sidecar.safety_events().begin(), sidecar.safety_events().end());
    safety_ok = chain.Verify(sealed);

    proto::ReplayReport replay;
    if (bundle.replayed_actions.empty()) {
        replay = PendingReplay(bundle.header.episode_id());
    } else if (bundle.bit_exact) {
        replay = FactualReplay(bundle.header.episode_id(), true,
                               bundle.recorded_actions, bundle.replayed_actions,
                               match_threshold);
    } else {
        replay = InterventionalReplay(bundle.header.episode_id(),
                                      bundle.intervention_improved,
                                      bundle.recorded_actions,
                                      bundle.replayed_actions, match_threshold);
    }
    *sidecar.mutable_replay() = replay;
    for (const auto& a : bundle.recorded_actions) {
        *sidecar.add_recorded_actions() = a;
    }
    for (const auto& a : bundle.replayed_actions) {
        *sidecar.add_replayed_actions() = a;
    }

    bool reroll_ok = sidecar.root_causes().empty();
    bool regression_ok = true;
    for (int i = 0; i < sidecar.root_causes_size(); ++i) {
        const auto& card = sidecar.root_causes(i);
        if (card.reroll_passed()) {
            reroll_ok = true;
        }
        if (card.delta_success_set() < -bundle.regression_limit) {
            regression_ok = false;
        }
    }

    const EpisodeMetric current = MetricFromSidecar(sidecar);
    std::vector<EpisodeMetric> fleet = bundle.history;
    fleet.push_back(current);
    const auto snap = ComputeMetrics(fleet);
    *sidecar.mutable_gate() = EvaluateGate(snap, replay, reroll_ok, safety_ok,
                                           regression_ok, coverage_floor);
    return sidecar;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
