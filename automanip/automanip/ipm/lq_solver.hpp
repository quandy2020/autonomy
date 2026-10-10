/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#include <ostream>
#include <vector>

#include <automanip/core/types.hpp>
#include <automanip/oc/oc_problem/ocp_size.hpp>

namespace automanip {
namespace ipm {

/**
 * Regularization for the discrete Riccati solve used by the IPM Newton step.
 * The condensed subproblem has linear dynamics and a quadratic cost. Inequality
 * constraints are already absorbed into that cost.
 */
struct LqSettings {
  scalar_t hessianReg = 1e-9;
};

std::ostream& operator<<(std::ostream& stream, const LqSettings& settings);

}  // namespace ipm

enum class LqStatus { SUCCESS, NAN_SOL };

/**
 * Solves a discrete linear-quadratic optimal control problem by Riccati recursion.
 * Optional constraint rows are equalities, f + dfdx * dx + dfdu * du = 0.
 * IpmSolver condenses inequalities into the cost and passes no rows. SqpSolver passes
 * state-input equalities when it does not project them out of the QP.
 */
class LqSolver {
 public:
  explicit LqSolver(OcpSize ocpSize = OcpSize(), const ipm::LqSettings& settings = ipm::LqSettings());

  void resize(OcpSize ocpSize);

  LqStatus solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                 std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                 vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool verbose = false);

  std::vector<ScalarFunctionQuadraticApproximation> getRiccatiCostToGo(const VectorFunctionLinearApproximation& dynamics0,
                                                                       const ScalarFunctionQuadraticApproximation& cost0) const;

  matrix_array_t getRiccatiFeedback(const VectorFunctionLinearApproximation& dynamics0,
                                    const ScalarFunctionQuadraticApproximation& cost0) const;

  vector_array_t getRiccatiFeedforward(const VectorFunctionLinearApproximation& dynamics0,
                                       const ScalarFunctionQuadraticApproximation& cost0) const;

 private:
  void verifySizes(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                   const std::vector<ScalarFunctionQuadraticApproximation>& cost) const;

  ipm::LqSettings settings_;
  OcpSize ocpSize_;
  bool solved_ = false;
  std::vector<ScalarFunctionQuadraticApproximation> costToGo_;
  matrix_array_t feedback_;
  vector_array_t feedforward_;
};

}  // namespace automanip
