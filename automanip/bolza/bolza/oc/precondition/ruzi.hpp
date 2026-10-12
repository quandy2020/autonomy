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
 * @file ruzi.hpp
 * @brief Calculates the pre-conditioning factors D, E, and c, and scale the input dynamics, and cost data in place and parallel.
 */


#include <Eigen/Sparse>

#include <bolza/core/types.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

#include "bolza/oc/oc_problem/ocp_size.hpp"

namespace bolza {
namespace precondition {

/**
 * Calculates the pre-conditioning factors D, E, and c, and scale the input dynamics, and cost data in place and parallel.
 *
 * There are a few pre-conditioning methods aiming to shape different aspects of the problem. To balance the performance and
 * computational effort, we choose a modified Ruzi equilibration Algorithm. Interested readers can find the original Ruiz
 * equilibration in: "Ruiz, D., 2001. A scaling algorithm to equilibrate both rows and columns norms in matrices".
 *
 * This pre-conditioning transforms the following minimization with z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}]
 *
 * min_{z} 1/2 z' H y + y' h
 * s.t.    G z = g
 *
 * to the follwoing one with y := inv(D) z
 *
 * min_{y} c/2 y' (D H D) y + c y' (D h)
 * s.t.    (E G D) y = E g
 *
 * The KKT matrices H, h, G, and g are defined as
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * @param [in] threadPool: The external thread pool.
 * @param [in] x0: The initial state.
 * @param [in] ocpSize: The size of the oc problem.
 * @param [in] iteration: Number of iterations.
 * @param [in,out] dynamics: The dynamics array of all time points.
 * @param [in,out] cost: The cost array of all time points.
 * @param [out] DOut: The matrix D decomposed for each time step.
 * @param [out] EOut: The matrix E decomposed for each time step.
 * @param [out] scalingVectors: Vector representation for the identity parts of the dynamics constraints inside the constraint matrix.
 *                               After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal components
 *                               of this type of matrix for every timestamp.
 * @param [out] cOut: Scaling factor c.
 */
void OcpDataInPlaceInParallel(ThreadPool& threadPool, const vector_t& x0, const OcpSize& ocpSize, const int iteration,
                              std::vector<VectorFunctionLinearApproximation>& dynamics,
                              std::vector<ScalarFunctionQuadraticApproximation>& cost, vector_array_t& DOut, vector_array_t& EOut,
                              vector_array_t& scalingVectors, scalar_t& cOut);

/**
 * Calculates the pre-conditioning factors D, E, and c, and scale the input dynamics, and cost data in place in place.
 *
 * There are a few pre-conditioning methods aiming to shape different aspects of the problem. To balance the performance and
 * computational effort, we choose a modified Ruzi equilibration Algorithm. Interested readers can find the original Ruiz
 * equilibration in: "Ruiz, D., 2001. A scaling algorithm to equilibrate both rows and columns norms in matrices".
 *
 * This pre-conditioning transforms the following minimization with z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}]
 *
 * min_{z} 1/2 z' H y + y' h
 * s.t.    G z = g
 *
 * to the follwoing one with y := inv(D) z
 *
 * min_{y} c/2 y' (D H D) y + c y' (D h)
 * s.t.    (E G D) y = E g
 *
 * The KKT matrices H, h, G, and g are defined as
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * For constructing H, h, G, and g, refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * @param [in] iteration: Number of iterations.
 * @param [in,out] H: The hessian matrix of the total cost.
 * @param [in,out] h: The jacobian vector of the total cost.
 * @param [in,out] G: The jacobian matrix of the constarinst.
 * @param [in,out] g: The constraints vector.
 * @param [out] DOut: The matrix D decomposed for each time step.
 * @param [out] EOut: The matrix E decomposed for each time step.
 * @param [out] cOut: Scaling factor c.
 */
void KktMatrixInPlace(int iteration, Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G, vector_t& g,
                      vector_t& DOut, vector_t& EOut, scalar_t& cOut);

/**
 * Scales the dynamics and cost array in place and construct scaling vector array from the given scaling factors E, D and c.
 *
 * @param [in] ocpSize: The size of the oc problem.
 * @param [in] D: Scaling factor D
 * @param [in] E: Scaling factor E
 * @param [in] c: Scaling factor c
 * @param [in,out] dynamics: The dynamics  array of all time points.
 * @param [in,out] cost: The cost array of all time points.
 * @param [out] scalingVectors: Vector representation for the identity parts of the dynamics inside the constraint matrix.
 *                               After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal
 *                               components of this type of matrix for every timestamp.
 */
void ScaleOcpData(const OcpSize& ocpSize, const vector_t& D, const vector_t& E, const scalar_t c,
                  std::vector<VectorFunctionLinearApproximation>& dynamics, std::vector<ScalarFunctionQuadraticApproximation>& cost,
                  std::vector<vector_t>& scalingVectors);

/**
 * Descales the solution. Note that the initial state is not considered a decision variable; therefore, it is not scaled.
 * This pre-conditioning transforms the following decision vector z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}] to y := inv(D) z.
 *
 * @param [in] D: Scaling factor D
 * @param [in,out] xTrajectory: The state trajectory of the length (N + 1).
 * @param [in,out] uTrajectory: The input trajectory of the length N.
 */
void DescaleSolution(const vector_array_t& D, vector_array_t& xTrajectory, vector_array_t& uTrajectory);

}  // namespace precondition
}  // namespace bolza