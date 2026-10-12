/**
 * @file single_thread_pipg.hpp
 * @brief First order primal-dual method for solving optimal control problem based on:.
 */

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

#include <Eigen/Sparse>

#include <bolza/core/misc/benchmark.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>

#include "bolza/slp/pipg/pipg_bounds.hpp"
#include "bolza/slp/pipg/pipg_settings.hpp"
#include "bolza/slp/pipg/pipg_solver_status.hpp"

namespace bolza {
namespace pipg {

/**
 * First order primal-dual method for solving optimal control problem based on:
 * "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * https://arxiv.org/abs/2009.06980
 *
 * z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * min  0.5 z' H z + z' h
 * s.t. G z = g
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
 * @param [in] settings: The PIPG settings.
 * @param [in] H: The hessian matrix of the total cost.
 * @param [in] h: The jacobian vector of the total cost.
 * @param [in] G: The jacobian matrix of the constarinst.
 * @param [in] g: The constraints vector.
 * @param [in] EInv: Inverse of the scaling factor E. Used to calculate un-sacled termination criteria.
 * @param [in] pipgBounds: The PipgBounds used to define the primal and dual stepsizes.
 * @param [out] stackedSolution: The concatenated state-input trajectories, z.
 * @return The solver status.
 */
SolverStatus SingleThreadPipg(const pipg::Settings& settings, const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                              const Eigen::SparseMatrix<scalar_t>& G, const vector_t& g, const vector_t& EInv, const PipgBounds& pipgBounds,
                              vector_t& stackedSolution);

}  // namespace pipg
}  // namespace bolza
