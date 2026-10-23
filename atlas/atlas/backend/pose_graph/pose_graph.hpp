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
 * @file pose_graph.hpp
 * @brief Shared pose graph. Factors are relative poses only: Pose3, or
 *        Similarity3 when the visual map scale is free.
 */

#ifndef ATLAS_BACKEND_POSE_GRAPH_HPP_
#define ATLAS_BACKEND_POSE_GRAPH_HPP_

#include <unordered_map>
#include <vector>

#include "common/keyframe.hpp"
#include "common/transform/rigid_transform.hpp"
#include "optimization/solver_factory.hpp"

namespace atlas {
namespace backend {

enum class PoseFactorKind {
  kPose3 = 0,
  kSimilarity3 = 1,
};

struct RelativePoseFactor {
  common::KeyframeId from_id = common::kInvalidKeyframeId;
  common::KeyframeId to_id = common::kInvalidKeyframeId;
  transform::Rigid3d relative_pose = transform::Rigid3d::Identity();
  double scale = 1.0;
  PoseFactorKind kind = PoseFactorKind::kPose3;
};

class PoseGraph {
 public:
  bool AddNode(common::KeyframeId id, const transform::Rigid3d& pose,
               bool fixed);
  void AddFactor(const RelativePoseFactor& factor);
  bool Optimize(optimization::GraphSolver* solver);
  bool Optimize(
      optimization::SolverKind kind = optimization::SolverKind::kGtsam);
  bool Pose(common::KeyframeId id, transform::Rigid3d* pose) const;

 private:
  struct Node {
    transform::Rigid3d pose = transform::Rigid3d::Identity();
    bool fixed = false;
  };

  std::unordered_map<common::KeyframeId, Node> nodes_;
  std::vector<RelativePoseFactor> factors_;
};

}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_POSE_GRAPH_HPP_
