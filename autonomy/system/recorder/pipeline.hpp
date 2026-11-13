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

#pragma once

/**
 * @file pipeline.hpp
 * @brief Offline L2–L6 evidence pipeline: graph, attribution, replay, metrics, gate.
 */

#include <vector>

#include "autonomy/system/recorder/episode.hpp"
#include "autonomy/system/recorder/metrics.hpp"
#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/root_cause.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Inputs for one episode's evidence pipeline.
 */
struct EpisodeBundle {
    proto::EpisodeHeader header;                        ///< Episode header.
    std::vector<BtTransition> transitions;              ///< BT transitions.
    std::vector<proto::TriggerEvent> triggers;          ///< L1 triggers.
    std::vector<proto::Observation> observations;       ///< Bound observations.
    std::vector<proto::ActionEvent> recorded_actions;   ///< Capture-time actions.
    std::vector<proto::ActionEvent> replayed_actions;   ///< Replay-time actions.
    std::vector<proto::SafetyEvent> safety_events;      ///< L4 chain events.
    std::vector<HistoricalEpisode> fail_set;            ///< Reroll fail history.
    std::vector<HistoricalEpisode> success_set;         ///< Reroll success history.
    CandidateFn reroll;                                 ///< Hypothesis matcher; may be empty.
    double regression_limit{0.05};                      ///< Max success-set hit rate.
    bool bit_exact{false};                              ///< Factual replay bit-exact flag.
    bool intervention_improved{false};                  ///< Interventional replay improved.
    std::vector<EpisodeMetric> history;                 ///< Prior episode metrics (fleet).
};

/**
 * @brief Run L2–L6 on a captured bundle and produce EpisodeSidecar.
 * @param [in] bundle: Episode inputs.
 * @param [in] match_threshold: L6 action-match threshold.
 * @param [in] coverage_floor: Minimum attribution coverage for the publish gate.
 * @return Filled sidecar (graph, root cause, BT, replay, gate).
 */
proto::EpisodeSidecar RunEvidencePipeline(const EpisodeBundle& bundle,
                                          double match_threshold,
                                          double coverage_floor);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
