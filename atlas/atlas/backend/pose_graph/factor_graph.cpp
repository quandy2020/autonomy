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

#include "backend/pose_graph/factor_graph.hpp"

#include <cmath>

namespace atlas {
namespace backend {
namespace pose_graph {

bool FactorGraph::AddPose(std::int64_t pose_identifier,
                          const transform::Rigid3d& pose, bool fixed) {
  if (pose_identifier < 0 || poses_.find(pose_identifier) != poses_.end()) {
    return false;
  }
  poses_.emplace(pose_identifier, PoseNode{pose, fixed});
  return true;
}

bool FactorGraph::SetPose(std::int64_t pose_identifier,
                          const transform::Rigid3d& pose) {
  const auto found = poses_.find(pose_identifier);
  if (found == poses_.end() || found->second.fixed) {
    return false;
  }
  found->second.pose = pose;
  return true;
}

void FactorGraph::AddPriorFactor(const PriorFactor& prior_factor) {
  prior_factors_.push_back(prior_factor);
}

void FactorGraph::AddBetweenFactor(const BetweenFactor& between_factor) {
  between_factors_.push_back(between_factor);
}

bool FactorGraph::IsValid(
    const std::unordered_set<std::int64_t>& known_pose_ids) const {
  for (const auto& entry : poses_) {
    if (known_pose_ids.count(entry.first) != 0 ||
        !entry.second.pose.translation().allFinite() ||
        !entry.second.pose.rotation().coeffs().allFinite() ||
        entry.second.pose.rotation().norm() < 1e-12) {
      return false;
    }
  }
  for (const PriorFactor& prior : prior_factors_) {
    const auto node = poses_.find(prior.pose_id);
    if ((node == poses_.end() && known_pose_ids.count(prior.pose_id) == 0) ||
        !std::isfinite(prior.rotation_weight) ||
        !std::isfinite(prior.translation_weight) ||
        prior.rotation_weight < 0.0 || prior.translation_weight < 0.0) {
      return false;
    }
    if (!prior.pose.translation().allFinite() ||
        !prior.pose.rotation().coeffs().allFinite() ||
        prior.pose.rotation().norm() < 1e-12) {
      return false;
    }
  }
  for (const BetweenFactor& between : between_factors_) {
    const auto source_pose = poses_.find(between.from_id);
    const auto target_pose = poses_.find(between.to_id);
    if ((source_pose == poses_.end() &&
         known_pose_ids.count(between.from_id) == 0) ||
        (target_pose == poses_.end() &&
         known_pose_ids.count(between.to_id) == 0) ||
        !std::isfinite(between.rotation_weight) ||
        !std::isfinite(between.translation_weight) ||
        !std::isfinite(between.scale) || between.rotation_weight < 0.0 ||
        between.translation_weight < 0.0 || between.scale <= 0.0 ||
        !between.relative_pose.translation().allFinite() ||
        !between.relative_pose.rotation().coeffs().allFinite() ||
        between.relative_pose.rotation().norm() < 1e-12) {
      return false;
    }
    if (between.diagonal_information_provided) {
      for (double information : between.diagonal_information) {
        if (!std::isfinite(information) || information < 0.0) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas
