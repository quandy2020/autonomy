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

#include "backend/backend_runner.hpp"

namespace argus {
namespace backend {

BackendRunner::BackendRunner(optimization::SolverKind kind)
    : solver_(optimization::CreateSolver(kind)) {
  if (solver_ == nullptr) {
    solver_ = optimization::CreateSolver(optimization::SolverKind::kEigen);
  }
}

bool BackendRunner::AddNode(common::KeyframeId id,
                            const transform::Rigid3d& pose, bool fixed) {
  return graph_.AddNode(id, pose, fixed);
}

void BackendRunner::AddFactor(const RelativePoseFactor& factor) {
  graph_.AddFactor(factor);
}

bool BackendRunner::Optimize() {
  return solver_ != nullptr && graph_.Optimize(solver_.get());
}

bool BackendRunner::Pose(common::KeyframeId id,
                         transform::Rigid3d* pose) const {
  return graph_.Pose(id, pose);
}

const char* BackendRunner::solver_name() const {
  return solver_ == nullptr ? "" : solver_->name();
}

}  // namespace backend
}  // namespace argus
