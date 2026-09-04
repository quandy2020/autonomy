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

#include "autonomy/perception/track/utils/follow_hypothesis_selection.hpp"

#include <algorithm>
#include <limits>

namespace autonomy::perception::track {
namespace {

double WrapAngleToPi(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double ComputeTargetBearing(const MotionPrimitiveHypothesis& hypothesis) {
  return std::atan2(hypothesis.terminal_y_m, hypothesis.terminal_x_m);
}

}  // namespace

int SelectFollowHypothesis(
    const std::vector<MotionPrimitiveHypothesis>& hypotheses,
    const FollowSelectionOptions& options, bool has_previous_target,
    double previous_target_yaw_rad) {
  if (hypotheses.empty()) {
    return -1;
  }

  std::vector<int> retained_indices;
  retained_indices.reserve(hypotheses.size());
  for (size_t index = 0; index < hypotheses.size(); ++index) {
    if (hypotheses[index].objectness_score >= options.objectness_threshold) {
      retained_indices.push_back(static_cast<int>(index));
    }
  }
  if (retained_indices.empty()) {
    int best_index = 0;
    for (size_t index = 1; index < hypotheses.size(); ++index) {
      if (hypotheses[index].objectness_score >
          hypotheses[best_index].objectness_score) {
        best_index = static_cast<int>(index);
      }
    }
    return hypotheses[best_index].objectness_score >
                   options.minimum_soft_objectness
               ? best_index
               : -1;
  }

  std::sort(retained_indices.begin(), retained_indices.end(),
            [&](int left, int right) {
              return hypotheses[left].objectness_score >
                     hypotheses[right].objectness_score;
            });

  std::vector<int> survivor_indices;
  for (int index : retained_indices) {
    const double yaw_rad = ComputeTargetBearing(hypotheses[index]);
    bool is_suppressed = false;
    for (int survivor_index : survivor_indices) {
      const double delta = std::abs(WrapAngleToPi(
          yaw_rad - ComputeTargetBearing(hypotheses[survivor_index])));
      if (delta < options.suppression_angle_rad) {
        is_suppressed = true;
        break;
      }
    }
    if (!is_suppressed) {
      survivor_indices.push_back(index);
    }
  }
  if (survivor_indices.empty()) {
    return -1;
  }

  int best_index = survivor_indices.front();
  double best_metric = std::numeric_limits<double>::infinity();
  for (int index : survivor_indices) {
    const auto& hypothesis = hypotheses[index];
    double metric = hypothesis.trajectory_cost -
                    options.objectness_ranking_weight *
                        hypothesis.objectness_score;
    if (has_previous_target) {
      const double delta = std::abs(WrapAngleToPi(
          ComputeTargetBearing(hypothesis) - previous_target_yaw_rad));
      metric += options.temporal_consistency_weight * delta;
    }
    if (metric < best_metric) {
      best_metric = metric;
      best_index = index;
    }
  }
  return best_index;
}

}  // namespace autonomy::perception::track
