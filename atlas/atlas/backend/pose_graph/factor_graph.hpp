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
 * @file factor_graph.hpp
 * @brief Atlas pose graph input consumed by the incremental solver.
 *
 * A between factor measures `T_from^{-1} T_to`. Similarity factors use
 * `scale` on the measured translation; the scale itself is not a variable.
 */

#ifndef ATLAS_BACKEND_POSE_GRAPH_FACTOR_GRAPH_HPP_
#define ATLAS_BACKEND_POSE_GRAPH_FACTOR_GRAPH_HPP_

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "backend/pose_graph/between_factor.hpp"
#include "backend/pose_graph/pose_node.hpp"
#include "backend/pose_graph/prior_factor.hpp"
#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace backend {
namespace pose_graph {

class FactorGraph {
 public:
  bool AddPose(std::int64_t pose_identifier, const transform::Rigid3d& pose,
               bool fixed);
  bool SetPose(std::int64_t pose_identifier, const transform::Rigid3d& pose);
  void AddPriorFactor(const PriorFactor& prior_factor);
  void AddBetweenFactor(const BetweenFactor& between_factor);

  const std::unordered_map<std::int64_t, PoseNode>& GetPoses() const {
    return poses_;
  }
  const std::vector<PriorFactor>& GetPriorFactors() const {
    return prior_factors_;
  }
  const std::vector<BetweenFactor>& GetBetweenFactors() const {
    return between_factors_;
  }
  bool ContainsPose(std::int64_t pose_identifier) const {
    return poses_.find(pose_identifier) != poses_.end();
  }
  /** @brief Checks this batch against nodes already in the solver. */
  bool IsValid(
      const std::unordered_set<std::int64_t>& known_pose_ids = {}) const;

 private:
  std::unordered_map<std::int64_t, PoseNode> poses_;
  std::vector<PriorFactor> prior_factors_;
  std::vector<BetweenFactor> between_factors_;
};

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_POSE_GRAPH_FACTOR_GRAPH_HPP_
