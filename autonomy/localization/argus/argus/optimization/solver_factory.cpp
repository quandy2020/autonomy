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

#include "optimization/solver_factory.hpp"

#include "optimization/g2o_graph_solver.hpp"
#include "optimization/gtsam_graph_solver.hpp"

namespace argus {
namespace optimization {

bool SolverAvailable(SolverKind kind) {
  switch (kind) {
    case SolverKind::kEigen:
      return true;
    case SolverKind::kGtsam:
      return GtsamSolverAvailable();
    case SolverKind::kG2o:
      return G2oSolverAvailable();
  }
  return false;
}

std::unique_ptr<GraphSolver> CreateSolver(SolverKind kind) {
  switch (kind) {
    case SolverKind::kEigen:
      return std::make_unique<EigenGraphSolver>();
    case SolverKind::kGtsam:
      return CreateGtsamSolver();
    case SolverKind::kG2o:
      return CreateG2oSolver();
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace argus
