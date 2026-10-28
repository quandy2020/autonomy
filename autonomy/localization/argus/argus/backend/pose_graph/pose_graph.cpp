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

#include "backend/pose_graph/pose_graph.hpp"

#include "optimization/factor_graph.hpp"

namespace argus {
namespace backend {

bool PoseGraph::AddNode(common::KeyframeId id, const transform::Rigid3d& pose,
                        bool fixed) {
  if (id == common::kInvalidKeyframeId || nodes_.find(id) != nodes_.end()) {
    return false;
  }
  nodes_.emplace(id, Node{pose, fixed});
  return true;
}

void PoseGraph::AddFactor(const RelativePoseFactor& factor) {
  factors_.push_back(factor);
}

bool PoseGraph::Optimize(optimization::SolverKind kind) {
  const std::unique_ptr<optimization::GraphSolver> solver =
      optimization::CreateSolver(kind);
  return solver != nullptr && Optimize(solver.get());
}

bool PoseGraph::Optimize(optimization::GraphSolver* solver) {
  if (solver == nullptr) {
    return false;
  }
  optimization::FactorGraph problem;
  for (const auto& entry : nodes_) {
    if (!problem.AddPose(entry.first, entry.second.pose, entry.second.fixed)) {
      return false;
    }
  }
  for (const RelativePoseFactor& factor : factors_) {
    optimization::BetweenFactor between;
    between.from = factor.from_id;
    between.to = factor.to_id;
    between.relative = factor.relative_pose;
    between.scale = factor.scale;
    between.similarity = factor.kind == PoseFactorKind::kSimilarity3;
    problem.AddBetween(between);
  }
  if (!solver->Optimize(&problem)) {
    return false;
  }
  for (auto& entry : nodes_) {
    const auto found = problem.poses().find(entry.first);
    if (found == problem.poses().end()) {
      return false;
    }
    entry.second.pose = found->second.pose;
  }
  return true;
}

bool PoseGraph::Pose(common::KeyframeId id, transform::Rigid3d* pose) const {
  const auto found = nodes_.find(id);
  if (pose == nullptr || found == nodes_.end()) {
    return false;
  }
  *pose = found->second.pose;
  return true;
}

}  // namespace backend
}  // namespace argus
