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

#include "optimization/factor_graph.hpp"

namespace atlas {
namespace optimization {

bool FactorGraph::AddPose(PoseId id, const transform::Rigid3d& pose,
                          bool fixed) {
  if (id < 0 || poses_.find(id) != poses_.end()) {
    return false;
  }
  poses_.emplace(id, PoseNode{pose, fixed});
  return true;
}

bool FactorGraph::SetPose(PoseId id, const transform::Rigid3d& pose) {
  const auto found = poses_.find(id);
  if (found == poses_.end() || found->second.fixed) {
    return false;
  }
  found->second.pose = pose;
  return true;
}

void FactorGraph::AddPrior(const PriorFactor& factor) {
  priors_.push_back(factor);
}

void FactorGraph::AddBetween(const BetweenFactor& factor) {
  betweens_.push_back(factor);
}

}  // namespace optimization
}  // namespace atlas
