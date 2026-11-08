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

/** @file pose_graph_solver.hpp @brief Incremental pose graph solver API. */

#ifndef ATLAS_BACKEND_POSE_GRAPH_POSE_GRAPH_SOLVER_HPP_
#define ATLAS_BACKEND_POSE_GRAPH_POSE_GRAPH_SOLVER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "backend/pose_graph/factor_graph.hpp"

namespace atlas {
namespace backend {
namespace pose_graph {

struct SolverOptions {
  // Upper bound on ISAM2 update calls for one batch, including insertion.
  int maximum_update_count = 20;
  double convergence_tolerance = 1e-8;
};

enum class SolverStatus {
  kSucceeded,
  kInvalidInput,
  kUnsupportedConstraint,
  kNumericalFailure,
};

struct UpdateResult {
  SolverStatus status = SolverStatus::kInvalidInput;
  int update_count = 0;
  std::string diagnostic_message;

  bool Succeeded() const { return status == SolverStatus::kSucceeded; }
};

struct SolverResult {
  SolverStatus status = SolverStatus::kInvalidInput;
  std::unordered_map<std::int64_t, transform::Rigid3d> optimized_poses;
  double final_cost = 0.0;
  std::string diagnostic_message;

  bool Succeeded() const { return status == SolverStatus::kSucceeded; }
};

class PoseGraphSolver {
 public:
  PoseGraphSolver();
  ~PoseGraphSolver();

  PoseGraphSolver(const PoseGraphSolver&) = delete;
  PoseGraphSolver& operator=(const PoseGraphSolver&) = delete;

  /** @brief Append only new nodes and factors to the persistent ISAM2 state. */
  UpdateResult Update(const FactorGraph& additions,
                      const SolverOptions& options = SolverOptions{});

  /** @brief Extract all poses and evaluate the current measurement cost. */
  SolverResult CalculateEstimate();

 private:
  struct State;
  std::unique_ptr<State> state_;
};

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_POSE_GRAPH_POSE_GRAPH_SOLVER_HPP_
