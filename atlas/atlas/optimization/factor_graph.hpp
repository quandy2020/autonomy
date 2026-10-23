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
 * @brief Pose graph shared by the Eigen, GTSAM, and g2o solvers.
 *
 * A between factor measures `T_from^{-1} T_to`. Similarity factors use
 * `scale` on the measured translation; the scale itself is not a variable.
 */

#ifndef ATLAS_OPTIMIZATION_FACTOR_GRAPH_HPP_
#define ATLAS_OPTIMIZATION_FACTOR_GRAPH_HPP_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace optimization {

using PoseId = std::int64_t;

struct PoseNode {
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  bool fixed = false;
};

struct PriorFactor {
  PoseId id = -1;
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  double rotation_weight = 1.0;
  double translation_weight = 1.0;
};

struct BetweenFactor {
  PoseId from = -1;
  PoseId to = -1;
  transform::Rigid3d relative = transform::Rigid3d::Identity();
  double rotation_weight = 1.0;
  double translation_weight = 1.0;
  double scale = 1.0;
  bool similarity = false;
};

class FactorGraph {
 public:
  bool AddPose(PoseId id, const transform::Rigid3d& pose, bool fixed);
  bool SetPose(PoseId id, const transform::Rigid3d& pose);
  void AddPrior(const PriorFactor& factor);
  void AddBetween(const BetweenFactor& factor);

  const std::unordered_map<PoseId, PoseNode>& poses() const { return poses_; }
  const std::vector<PriorFactor>& priors() const { return priors_; }
  const std::vector<BetweenFactor>& betweens() const { return betweens_; }
  bool Contains(PoseId id) const { return poses_.find(id) != poses_.end(); }

 private:
  std::unordered_map<PoseId, PoseNode> poses_;
  std::vector<PriorFactor> priors_;
  std::vector<BetweenFactor> betweens_;
};

}  // namespace optimization
}  // namespace atlas

#endif  // ATLAS_OPTIMIZATION_FACTOR_GRAPH_HPP_
