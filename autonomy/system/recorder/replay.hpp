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
 * @file replay.hpp
 * @brief L6 replay verdicts: factual vs interventional, plus capture-time pending.
 */

#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Fraction of recorded actions that match replayed actions within tolerance.
 * @param [in] recorded: Capture-time action sequence.
 * @param [in] replayed: Replay-time action sequence.
 * @param [in] param_tolerance: Absolute vx/wz tolerance.
 * @return Match ratio in [0, 1].
 */
double ActionMatch(const std::vector<proto::ActionEvent>& recorded,
                   const std::vector<proto::ActionEvent>& replayed,
                   double param_tolerance);

/**
 * @brief Factual replay report: same policy, expect match (optionally bit-exact).
 * @param [in] episode_id: Episode id.
 * @param [in] bit_exact: Require identical action bytes.
 * @param [in] recorded: Capture-time actions.
 * @param [in] replayed: Replay-time actions.
 * @param [in] match_threshold: Pass threshold on ActionMatch.
 * @return ReplayReport.
 */
proto::ReplayReport FactualReplay(const std::string& episode_id,
                                  bool bit_exact,
                                  const std::vector<proto::ActionEvent>& recorded,
                                  const std::vector<proto::ActionEvent>& replayed,
                                  double match_threshold);

/**
 * @brief Interventional replay report: modified policy; improved means closer to intent.
 * @param [in] episode_id: Episode id.
 * @param [in] improved: Whether the intervention improved the outcome.
 * @param [in] recorded: Capture-time actions.
 * @param [in] replayed: Replay-time actions.
 * @param [in] match_threshold: Pass threshold (used as a similarity floor).
 * @return ReplayReport.
 */
proto::ReplayReport InterventionalReplay(
    const std::string& episode_id, bool improved,
    const std::vector<proto::ActionEvent>& recorded,
    const std::vector<proto::ActionEvent>& replayed, double match_threshold);

/**
 * @brief Capture-time placeholder: no independent replayed action log yet.
 * @param [in] episode_id: Episode id.
 * @return ReplayReport with pending status.
 */
proto::ReplayReport PendingReplay(const std::string& episode_id);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
