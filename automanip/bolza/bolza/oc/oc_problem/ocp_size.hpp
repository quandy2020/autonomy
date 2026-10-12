/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file ocp_size.hpp
 * @brief Size of the optimal control problem to be solved with the HpipmInterface.
 */


#include <vector>

#include <bolza/core/types.hpp>

namespace bolza {
/**
 * Size of the optimal control problem to be solved with the HpipmInterface
 *
 * The number of stages (N) is defined as the number of transitions, the number of nodes, including terminal node, is therefore (N+1).
 *
 * For example, a problem with:
 * (t0, x0) -> dynamics driven by u0 ->  (t1, x1) -> dynamics driven by u1 -> (t2, x2)
 * Has 2 stages, and 3 nodes.
 *
 * HPIPM requires all sizes to be declared per node. All vectors therefore need to be of size N+1.
 * For the optimal control problems we are currently solving, we therefore set numInput(N+1) = 0.
 */
struct OcpSize {
  int numStages;                            // Number of stages (N), all vectors below must be of size N+1
  std::vector<int> numInputs;               // Number of inputs
  std::vector<int> numStates;               // Number of states
  std::vector<int> numInputBoxConstraints;  // Number of input box inequality constraints
  std::vector<int> numStateBoxConstraints;  // Number of state box inequality constraints
  std::vector<int> numIneqConstraints;      // Number of general inequality constraints
  std::vector<int> numInputBoxSlack;        // Number of slack variables for input box inequalities
  std::vector<int> numStateBoxSlack;        // Number of slack variables for state box inequalities
  std::vector<int> numIneqSlack;            // Number of slack variables for general inequalities

  /** Constructor for N stages with constant state and inputs and without constraints */
  explicit OcpSize(int N = 0, int nx = 0, int nu = 0)
      /**
       * @brief Num Stages.
       * @return Result value.
       */
      : numStages(N),
        numStates(N + 1, nx),
        numInputs(N + 1, nu),
        numInputBoxConstraints(N + 1, 0),
        numStateBoxConstraints(N + 1, 0),
        numIneqConstraints(N + 1, 0),
        numInputBoxSlack(N + 1, 0),
        numStateBoxSlack(N + 1, 0),
        numIneqSlack(N + 1, 0) {
    /**
     * @brief Back.
     * @return Result value.
     */
    numInputs.back() = 0;
  }
};

bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept;

/**
 * Extract sizes based on the problem data
 *
 * @param [in] dynamics: Linearized approximation of the discrete dynamics.
 * @param [in] cost: Quadratic approximation of the cost.
 * @param [in] constraints: Linearized approximation of constraints, all constraints are mapped to inequality constraints in HPIPM.
 * @return Derived sizes
 */
OcpSize ExtractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints);

}  // namespace bolza