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
 * @file
 * @brief Objectness-aware angular non-maximum suppression for human following.
 */

#pragma once

#include <cmath>
#include <vector>

#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/common/types.hpp"

namespace autonomy::perception::track {

/**
 * @struct autonomy::perception::track::FollowSelectionOptions
 * @brief Thresholds for objectness filtering and angular suppression.
 */
struct FollowSelectionOptions {
  double objectness_threshold{defaults::kObjectnessThreshold};
  double suppression_angle_rad{
      defaults::kSuppressionAngleDeg * M_PI / 180.0};
  double temporal_consistency_weight{0.25};
  // Soft fallback accepts a peak below |objectness_threshold| only if above this.
  double minimum_soft_objectness{kHeuristicBackgroundObjectness};
  // Ranking metric: trajectory_cost - weight * objectness_score (+ temporal).
  double objectness_ranking_weight{0.5};
};

/**
 * @brief Selects the best lattice hypothesis for human following.
 * @return Index into |hypotheses|, or -1 when none are usable.
 */
int SelectFollowHypothesis(
    const std::vector<MotionPrimitiveHypothesis>& hypotheses,
    const FollowSelectionOptions& options, bool has_previous_target,
    double previous_target_yaw_rad);

}  // namespace autonomy::perception::track
