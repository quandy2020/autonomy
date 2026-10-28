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
 * @file g2o_graph_solver.hpp
 * @brief g2o Levenberg-Marquardt backend for `FactorGraph`.
 *
 * Similarity edges are not represented by g2o's SE3 types. `Optimize` rejects
 * a graph that contains one.
 */

#ifndef ARGUS_OPTIMIZATION_G2O_GRAPH_SOLVER_HPP_
#define ARGUS_OPTIMIZATION_G2O_GRAPH_SOLVER_HPP_

#include <memory>

#include "optimization/graph_solver.hpp"

namespace argus {
namespace optimization {

bool G2oSolverAvailable();
std::unique_ptr<GraphSolver> CreateG2oSolver();

}  // namespace optimization
}  // namespace argus

#endif  // ARGUS_OPTIMIZATION_G2O_GRAPH_SOLVER_HPP_
