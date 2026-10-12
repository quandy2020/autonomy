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
 * @file ocp_to_kkt.hpp
 * @brief Constructs concatenated linear approximation of the constraints from the dynamics and constraints arrays w.r.t.
 */


#include <Eigen/Sparse>

#include <bolza/core/types.hpp>

#include "bolza/oc/oc_problem/ocp_size.hpp"

namespace bolza {

/**
 * Constructs concatenated linear approximation of the constraints from the dynamics and constraints arrays w.r.t.
 * Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G Z = g
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] x0: The initial state.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param [in] scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                               they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param [out] res: The resulting constraints approxmation. Here we misused dfdx field to store the jacobian w.r.t. Z.
 */
void GetConstraintMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                         const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                         VectorFunctionLinearApproximation& res);

/**
 * Constructs concatenated linear approximation of the constraints from the dynamics and constraints arrays w.r.t.
 * Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G Z = g
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] x0: The initial state.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param [in] scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                               they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param [out] G: The jacobian of the concatenated constraints w.r.t. Z.
 * @param [out] g: The concatenated constraints value.
 */
void GetConstraintMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                               Eigen::SparseMatrix<scalar_t>& G, vector_t& g);

/**
 * Constructs concatenated quadratic approximation of the total cost w.r.t. Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 Z' H Z + Z' h + h0
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] x0: The initial state.
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @param [out] res: The resulting cost approxmation. Here we misused dfdx and dfdxx fields to store the jacobian and the hessian
 *                  matrices w.r.t. Z.
 */
void GetCostMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   ScalarFunctionQuadraticApproximation& res);

/**
 * Constructs concatenated the jacobian and the hessian matrices of the total cost w.r.t. Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 Z' H Z + Z' h + h0
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] x0: The initial state.
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @param [out] H: The concatenated hessian matrix w.r.t. Z.
 * @param [out] h: The concatenated jacobian vector w.r.t. Z.
 */
void GetCostMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         Eigen::SparseMatrix<scalar_t>& H, vector_t& h);

/**
 * Deserializes the stacked solution to state-input trajecotries. Note that the initial state is not part of the stacked solution.
 *
 * @param [in] ocpSize: Optimal control problem sizes.
 * @param [in] stackedSolution: Defined as [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 * @param [in] x0: The initial state.
 * @param [out] xTrajectory: State tarjectory.
 * @param [out] uTrajectory: Input trajecotry.
 */
void ToOcpSolution(const OcpSize& ocpSize, const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
                   vector_array_t& uTrajectory);

/**
 * @brief Serializes the state-input trajecotries. Note that the initial state is not considered as a decision variable.
 * @param [in] xTrajectory: State tarjectory.
 * @param [in] uTrajectory: U Trajectory.
 * @param [out] stackedSolution: [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 */
void ToKktSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution);

}  // namespace bolza