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
 * @file root_cause.hpp
 * @brief L2 root-cause ranking: walk predecessors of the last FAILURE; optional reroll.
 */

#include <functional>
#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief One historical episode used as a reroll / regression reference.
 */
struct HistoricalEpisode {
    std::string scene_id;  ///< Scene identifier.
    bool success{false};   ///< True if that episode succeeded.
};

/**
 * @brief Predicate: does this historical episode match the candidate hypothesis.
 */
using CandidateFn = std::function<bool(const HistoricalEpisode&)>;

/**
 * @brief Rank root-cause cards by walking predecessors of the last FAILURE node.
 * @param [in] graph: L2 event graph.
 * @return Ordered RootCauseCard list (may be empty).
 */
std::vector<proto::RootCauseCard> RankRootCauses(const proto::EventGraph& graph);

/**
 * @brief Test a hypothesis against fail/success history; reject on regression.
 * @param [in] candidate: Match predicate over historical episodes.
 * @param [in] fail_set: Known-fail episodes.
 * @param [in] success_set: Known-success episodes.
 * @param [in] regression_limit: Max allowed success-set hit rate.
 * @param [out] card: Card to annotate (accepted / rejected).
 * @return True if the hypothesis is kept.
 */
bool RerollHypothesis(const CandidateFn& candidate,
                      const std::vector<HistoricalEpisode>& fail_set,
                      const std::vector<HistoricalEpisode>& success_set,
                      double regression_limit, proto::RootCauseCard* card);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
