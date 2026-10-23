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
 * @file backend_runner.hpp
 * @brief Pose-graph backend. Loop edges arrive as `RelativePoseFactor`.
 */

#ifndef ATLAS_BACKEND_BACKEND_RUNNER_HPP_
#define ATLAS_BACKEND_BACKEND_RUNNER_HPP_

#include <memory>

#include "backend/pose_graph/pose_graph.hpp"

namespace atlas {
namespace backend {

class BackendRunner {
 public:
  explicit BackendRunner(
      optimization::SolverKind kind = optimization::SolverKind::kGtsam);

  bool AddNode(common::KeyframeId id, const transform::Rigid3d& pose,
               bool fixed);
  void AddFactor(const RelativePoseFactor& factor);
  bool Optimize();
  bool Pose(common::KeyframeId id, transform::Rigid3d* pose) const;
  const char* solver_name() const;

 private:
  std::unique_ptr<optimization::GraphSolver> solver_;
  PoseGraph graph_;
};

}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_BACKEND_RUNNER_HPP_
