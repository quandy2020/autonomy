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
 * @file qp_solver.hpp
 * @brief Solves the discretized linear quadratic optimal control problem by constructing a dense QP and inverting the full KKT system.
 */


#include "bolza/qp/qp_solver_types.hpp"
#include "bolza/qp/qp_trajectories.hpp"

namespace bolza {
namespace qp_solver {

/**
 * Solves the discretized linear quadratic optimal control problem by constructing a dense QP and inverting the full KKT system.
 * The decision vector is defined as w = [dx[0], du[0], dx[1],  du[1], ..., dx[N]]
 *
 * @param [in] lqApproximation: vector of stage-wise discrete quadratic cost and linear dynamics
 * @param [in] dx0: initial state deviation from the nominal trajectories.
 * @return trajectory of state and inputs (in relative coordinates), .i.e. dx(t), du(t)
 */
std::pair<vector_array_t, vector_array_t> SolveLinearQuadraticProblem(const std::vector<LinearQuadraticStage>& lqApproximation,
                                                                      const vector_t& dx0);

/**
 * Constructs the matrix of stacked dynamic constraints A w + b = 0
 *
 * A = [-I  *
 *       C  D  *  *
 *       A  B -I  *
 *       *  *  C  D  *  *
 *       *  *  A  B -I  *
 *       *  *  *  *  C  D  *
 *       *  *  *  *  A  B -I ]
 *       *  *  *  *  *  *  C ]
 *
 * b = [x0; e[0]; b[0]; ... e[N-1]; b[N-1]; e[N]]
 *
 * @param [in] lqp: linear quadratic problem.
 * @param [in] dx0: initial state deviation from the nominal trajectories.
 * @param [in] numConstraints: number of rows in A
 * @param [in] numDecisionVariables: size of w
 * @return linear constraints in w, where w is the vector of decision variables
 */
VectorFunctionLinearApproximation GetConstraintMatrices(const std::vector<LinearQuadraticStage>& lqp, const vector_t& dx0,
                                                        int numConstraints, int numDecisionVariables);

/**
 * @brief Constructs a matrix of stacked cost functions  1/2 w' H w + g' w.
 *
 * H = [ Q  P'
 *
 * P  R
 *
 * Q  P'
 *
 * P  R
 *
 * Qf ]
 *
 * g = [q[0]; r[0]; q[1]; r[1]; ... qf ]
 * @param [in] lqp: Lqp.
 * @param [in] numDecisionVariables: size of w
 * @return quadratic cost function in w, where w is the vector of decision variables
 */
ScalarFunctionQuadraticApproximation GetCostMatrices(const std::vector<LinearQuadraticStage>& lqp, int numDecisionVariables);

/**
 * Constructs the equality constrained QP from the discretized linear quadratic control problem.
 * @param [in] lqp: linear quadratic problem.
 * @param [in] dx0: initial state deviation from the nominal trajectories.
 * @return { cost matrices, constraint matrices }
 */
std::pair<ScalarFunctionQuadraticApproximation, VectorFunctionLinearApproximation> GetDenseQp(const std::vector<LinearQuadraticStage>& lqp,
                                                                                              const vector_t& dx0);

/**
 * @brief Solves the equality constrained QP.
 *
 * min_w  1/2 w' H w + g' w
 *
 * s.t. A w + b = 0
 *
 * Assumes H is positive definite, rows of A are linearly independent.
 * @param [in] cost: Cost.
 * @param [in] constraints: Constraints.
 * @return {w, lambda} at the solution, where w is the vector of decision variables, and lambda is the vector of lagrange multipliers
 */
std::pair<vector_t, vector_t> SolveDenseQp(const ScalarFunctionQuadraticApproximation& cost,
                                           const VectorFunctionLinearApproximation& constraints);

/**
 * Reconstructs the optimal state and input trajectory recursively based on the full qp solution vector
 * @param [in] numStates: number of states per stage
 * @param [in] numInputs: number of inputs per stage
 * @param [in] w: the vector of decision variables
 * @return { state_trajectory, input_trajectory }
 */
std::pair<vector_array_t, vector_array_t> GetStateAndInputTrajectory(const std::vector<int>& numStates, const std::vector<int>& numInputs,
                                                                     const vector_t& w);

}  // namespace qp_solver
}  // namespace bolza
