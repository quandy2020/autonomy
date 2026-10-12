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
 * @file helpers.hpp
 * @brief Computes the upper bound of eigenvalues for the total cost hessian matrix. The bound is computed based on the Gershgorin Circle Theorem.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>

namespace bolza {
namespace slp {

/**
 * Computes the upper bound of eigenvalues for the total cost hessian matrix. The bound is computed based on the Gershgorin Circle Theorem.
 *
 * As the hessian matrix is a real symmetric matrix, all eigenvalues are real. Therefore, instead of estimating the
 * upper bound on a complex plane, we can estimate the bound along a 1D real number axis. The estimated upper bound is
 * defined as max_{i} [H_{ii} + R_{i}], where R_{i} is the radius of i'th Gershgorin discs. The Gershgorin discs radius is
 * defined ad R_{i} = sum_{j \neq i} |H_{ij}|. Thus the upper bound of eigenvalues yields from max_{i}sum{j} |H_{ij}|.
 *
 * Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 z' H z + z' h + h0
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
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @return The upper bound of eigenvalues for H.
 */
scalar_t HessianEigenvaluesUpperBound(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost);

/**
 * @brief Computes the upper bound of eigenvalues for the matrix G G' ( in parallel). The bound is computed based on the Gershgorin Circle Theorem.
 *
 * As the G G' matrix is a real symmetric matrix, all eigenvalues are real. Therefore, instead of estimating the
 *
 * upper bound on a complex plane, we can estimate the bound along a 1D real number axis. The estimated upper bound is
 *
 * defined as max_{i} [GG'_{ii} + R_{i}], where R_{i} is the radius of i'th Gershgorin discs. The Gershgorin discs radius is
 *
 * defined ad R_{i} = sum_{j \neq i} |GG'_{ij}|. Thus the upper bound of eigenvalues yields from max_{i}sum{j} |GG'_{ij}|.
 *
 * Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G z = g
 *
 * G = [-B0  I
 *
 * *  -A1 -B1   I
 *
 * *   *   *   -An -Bn  I
 *
 * D0  0
 *
 * *   C1  D1   0
 *
 * *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param [in] threadPool: The thread pool.
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraintsPtr: Constraints Ptr.
 * @param [in] scalingVectorsPtr: Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
 * @return The upper bound of eigenvalues for G G'.
 */
scalar_t GGTEigenvaluesUpperBound(ThreadPool& threadPool, const OcpSize& ocpSize,
                                  const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                  const vector_array_t* scalingVectorsPtr);

/**
 * Computes the row-wise absolute sum of the cost hessian matrix, H. Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * totalCost = 0.5 z' H z + z' h + h0
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
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @return The absolute sum of rows of matrix H.
 */
vector_t HessianAbsRowSum(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost);

/**
 * @brief Computes the row-wise absolute sum of matrix G G' in parallel. Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G z = g
 *
 * G = [-B0  I
 *
 * *  -A1 -B1   I
 *
 * *   *   *   -An -Bn  I
 *
 * D0  0
 *
 * *   C1  D1   0
 *
 * *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param [in] threadPool: The thread pool.
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraintsPtr: Constraints Ptr.
 * @param [in] scalingVectorsPtr: Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
 * @return The absolute sum of rows of matrix G G'.
 */
vector_t GGTAbsRowSumInParallel(ThreadPool& threadPool, const OcpSize& ocpSize,
                                const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                const vector_array_t* scalingVectorsPtr);

}  // namespace slp
}  // namespace bolza