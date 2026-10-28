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
 * @file gtsam_graph_solver.hpp
 * @brief GTSAM Levenberg-Marquardt backend for `FactorGraph`.
 */

#ifndef ARGUS_OPTIMIZATION_GTSAM_GRAPH_SOLVER_HPP_
#define ARGUS_OPTIMIZATION_GTSAM_GRAPH_SOLVER_HPP_

#include <memory>

#include "optimization/graph_solver.hpp"

namespace argus {
namespace optimization {

bool GtsamSolverAvailable();
std::unique_ptr<GraphSolver> CreateGtsamSolver();

}  // namespace optimization
}  // namespace argus

#endif  // ARGUS_OPTIMIZATION_GTSAM_GRAPH_SOLVER_HPP_
