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
 * @file graph_solver.hpp
 * @brief Solver interface implemented by Eigen, GTSAM, and g2o.
 */

#ifndef ARGUS_OPTIMIZATION_GRAPH_SOLVER_HPP_
#define ARGUS_OPTIMIZATION_GRAPH_SOLVER_HPP_

#include "optimization/factor_graph.hpp"

namespace argus {
namespace optimization {

class GraphSolver {
 public:
  virtual ~GraphSolver() = default;
  virtual const char* name() const = 0;
  virtual bool Optimize(FactorGraph* graph) = 0;
};

class EigenGraphSolver : public GraphSolver {
 public:
  const char* name() const override { return "eigen"; }
  bool Optimize(FactorGraph* graph) override;
};

}  // namespace optimization
}  // namespace argus

#endif  // ARGUS_OPTIMIZATION_GRAPH_SOLVER_HPP_
