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
 * @file lagrangian_evaluation.hpp
 * @brief Computes the SSE of the residual in the dual feasibilities.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace multiple_shooting {

/**
 * Computes the SSE of the residual in the dual feasibilities.
 *
 * @param [in] lagrangian: Quadratic approximation of the Lagrangian for a single node.
 * @return SSE of the residual in the dual feasibilities
 */
inline scalar_t EvaluateDualFeasibilities(const ScalarFunctionQuadraticApproximation& lagrangian) {
  /**
   * @brief Squared Norm.
   * @return Result value.
   */
  return lagrangian.dfdx.squaredNorm() + lagrangian.dfdu.squaredNorm();
}

/**
 * Evaluates the quadratic approximation of the Lagrangian for a single intermediate node.
 *
 * @param [in] lmd: Costate at start of the interval
 * @param [in] lmd_next: Costate at the end of the interval
 * @param [in] nu: Lagrange multiplier of the projection constraint.
 * @param [in] cost: Quadratic approximation of the cost.
 * @param [in] dynamics: Linear approximation of the dynamics
 * @param [in] stateInputEqConstraints: Linear approximation of the state-input equality constraints
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation EvaluateLagrangianIntermediateNode(const vector_t& lmd, const vector_t& lmd_next, const vector_t& nu,
                                                                        ScalarFunctionQuadraticApproximation&& cost,
                                                                        const VectorFunctionLinearApproximation& dynamics,
                                                                        const VectorFunctionLinearApproximation& stateInputEqConstraints);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the terminal node.
 *
 * @param [in] lmd: Costate at start of the interval
 * @param [in] cost: Quadratic approximation of the cost.
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation EvaluateLagrangianTerminalNode(const vector_t& lmd, ScalarFunctionQuadraticApproximation&& cost);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the event node.
 *
 * @param [in] lmd: Costate at start of the interval
 * @param [in] lmd_next: Costate at the end of the the interval
 * @param [in] cost: Quadratic approximation of the cost.
 * @param [in] dynamics: Dynamics
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation EvaluateLagrangianEventNode(const vector_t& lmd, const vector_t& lmd_next,
                                                                 ScalarFunctionQuadraticApproximation&& cost,
                                                                 const VectorFunctionLinearApproximation& dynamics);

}  // namespace multiple_shooting
}  // namespace bolza