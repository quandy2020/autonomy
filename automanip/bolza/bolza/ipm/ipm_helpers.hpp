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
 * @file ipm_helpers.hpp
 * @brief Removes the Newton directions of slack and dual variables and the linearized inequality constraints are removed from the linear system.
 */


#include <bolza/core/types.hpp>
#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/oc_data/dual_solution.hpp>
#include <bolza/oc/oc_data/time_discretization.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {
namespace ipm {

/**
 * Removes the Newton directions of slack and dual variables and the linearized inequality constraints are removed from the linear system
 * equations for the Newton step computation. These terms are considered in the quadratic approximation of the Lagrangian.
 *
 * @param [in] barrierParam: The barrier parameter of the interior point method.
 * @param [in] slack: The slack variable associated with the inequality constraints.
 * @param [in] dual: The dual variable associated with the inequality constraints.
 * @param [in] ineqConstraints: Linear approximation of the inequality constraints.
 * @param [in,out] lagrangian: Quadratic approximation of the Lagrangian.
 */
void CondenseIneqConstraints(scalar_t barrierParam, const vector_t& slack, const vector_t& dual,
                             const VectorFunctionLinearApproximation& ineqConstraints, ScalarFunctionQuadraticApproximation& lagrangian);

/**
 * Computes the SSE of the residual in the perturbed complementary slackness.
 *
 * @param [in] barrierParam: The barrier parameter of the interior point method.
 * @param [in] slack: The slack variable associated with the inequality constraints.
 * @param [in] dual: The dual variable associated with the inequality constraints.
 * @return SSE of the residual in the perturbed complementary slackness
 */
inline scalar_t EvaluateComplementarySlackness(scalar_t barrierParam, const vector_t& slack, const vector_t& dual) {
  return (slack.array() * dual.array() - barrierParam).matrix().squaredNorm();
}

/**
 * Retrieves the Newton directions of the slack variable associated with state-input inequality constraints.
 *
 * @param [in] stateInputIneqConstraints: State-input inequality constraints
 * @param [in] dx: Newton direction of the state
 * @param [in] du: Newton direction of the input
 * @param [in] barrierParam: The barrier parameter of the interior point method.
 * @param [in] slackStateInputIneq: The slack variable associated with the state-input inequality constraints.
 * @return Newton directions of the slack variable.
 */
vector_t RetrieveSlackDirection(const VectorFunctionLinearApproximation& stateInputIneqConstraints, const vector_t& dx, const vector_t& du,
                                scalar_t barrierParam, const vector_t& slackStateInputIneq);

/**
 * Retrieves the Newton directions of the slack variable associated with state-only inequality constraints.
 *
 * @param [in] stateIneqConstraints: State-only inequality constraints
 * @param [in] dx: Newton direction of the state
 * @param [in] barrierParam: The barrier parameter of the interior point method.
 * @param [in] slackStateIneq: The slack variable associated with the state-only inequality constraints.
 * @return Newton directions of the slack variable.
 */
vector_t RetrieveSlackDirection(const VectorFunctionLinearApproximation& stateIneqConstraints, const vector_t& dx, scalar_t barrierParam,
                                const vector_t& slackStateIneq);

/**
 * Retrieves the Newton directions of the dual variable.
 *
 * @param [in] barrierParam: The barrier parameter of the interior point method.
 * @param [in] slack: The slack variable associated with the inequality constraints.
 * @param [in] dual: The dual variable associated with the inequality constraints.
 * @param [in] slackDirection: The Newton direction of the slack variable.
 * @return Newton directions of the dual variable.
 */
vector_t RetrieveDualDirection(scalar_t barrierParam, const vector_t& slack, const vector_t& dual, const vector_t& slackDirection);

/**
 * @brief Computes the step size via fraction-to-boundary-rule, which is introduced in the IPOPT's implementaion paper,.
 *
 * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
 *
 * https://link.springer.com/article/10.1007/s10107-004-0559-y
 * @param [in] v: A variable (slack or dual).
 * @param [in] dv: The serach direction of the variable (slack dirction or dual dirction).
 * @param [in] marginRate: Margin rate to avoid the slack or dual variables becoming too close to 0. Typical values are 0.95 or 0.995.
 * @return Result value.
 */
scalar_t FractionToBoundaryStepSize(const vector_t& v, const vector_t& dv, scalar_t marginRate = 0.995);

/**
 * Convert the optimized slack or dual trajectories as a DualSolution.
 *
 * @param [in] time: The time discretization.
 * @param [in] constraintsSize: The constraint tems size.
 * @param [in] stateIneq: The slack/dual variable trajectory of the state inequality constraints.
 * @param [in] stateInputIneq: The slack/dual variable trajectory of the state-input inequality constraints.
 * @return A dual solution.
 */
DualSolution ToDualSolution(const std::vector<AnnotatedTime>& time, const std::vector<multiple_shooting::ConstraintsSize>& constraintsSize,
                            const vector_array_t& stateIneq, const vector_array_t& stateInputIneq);

/**
 * Extracts slack/dual variables of the state-only and state-input constraints from a MultiplierCollection
 *
 * @param [in] multiplierCollection: The MultiplierCollection.
 * @return slack/dual variables of the state-only (first) and state-input constraints (second).
 */
std::pair<vector_t, vector_t> FromMultiplierCollection(const MultiplierCollection& multiplierCollection);

}  // namespace ipm
}  // namespace bolza