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
 * @file pipg_solver.hpp
 * @brief Constructor.
 */


#include <string>

#include <Eigen/Sparse>

#include <bolza/core/types.hpp>
#include <bolza/core/misc/benchmark.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>

#include "bolza/slp/pipg/pipg_bounds.hpp"
#include "bolza/slp/pipg/pipg_settings.hpp"
#include "bolza/slp/pipg/pipg_solver_status.hpp"

namespace bolza {

/*
 * First order primal-dual method for solving optimal control problem based on:
 * "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * https://arxiv.org/abs/2009.06980
 */
class PipgSolver {
 public:
  /**
   * @brief Constructor.
   * @param [in] settings: Settings.
   */
  explicit PipgSolver(pipg::Settings settings);

  /**
   * Solve the optimal control in parallel.
   *
   * @param [in] threadPool: The external thread pool.
   * @param [in] x0: Initial state
   * @param [in] dynamics: Dynamics array.
   * @param [in] cost: Cost array.
   * @param [in] constraints: Constraints array. Pass nullptr for an unconstrained problem.
   * @param [in] scalingVectors: Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
   *                              they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
   * @param [in] EInv: Inverse of the scaling factor E. Used to calculate un-sacled termination criteria.
   * @param [in] pipgBounds: The PipgBounds used to define the primal and dual stepsizes.
   * @param [out] xTrajectory: The optimized state trajectory.
   * @param [out] uTrajectory: The optimized input trajectory.
   * @return The solver status.
   */
  pipg::SolverStatus solve(ThreadPool& threadPool, const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                           const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                           const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t& scalingVectors,
                           const vector_array_t* EInv, const pipg::PipgBounds& pipgBounds, vector_array_t& xTrajectory,
                           vector_array_t& uTrajectory);

  /**
   * @brief Resize.
   * @param [in] size: Size.
   */
  void resize(const OcpSize& size);

  /**
   * @brief Get Num Decision Variables.
   * @return Result value.
   */
  int GetNumDecisionVariables() const { return numDecisionVariables_; }
  /**
   * @brief Get Num Dynamics Constraints.
   * @return Result value.
   */
  int GetNumDynamicsConstraints() const { return numDynamicsConstraints_; }

  /**
   * @brief Size.
   * @return Result value.
   */
  const OcpSize& size() const { return ocpSize_; }
  /**
   * @brief Settings.
   * @return Result value.
   */
  const pipg::Settings& Settings() const { return settings_; }

 private:
  /**
   * @brief Verify Sizes.
   * @param [in] dynamics: Dynamics.
   * @param [in] cost: Cost.
   * @param [in] constraints: Constraints.
   */
  void VerifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                   const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   const std::vector<VectorFunctionLinearApproximation>* constraints) const;

  /**
   * @brief Verify Ocp Size.
   * @param [in] ocpSize: Ocp Size.
   */
  void VerifyOcpSize(const OcpSize& ocpSize) const;

  // Settings
  const pipg::Settings settings_;

  // Problem size
  OcpSize ocpSize_;
  int numDecisionVariables_;
  int numDynamicsConstraints_;

  // Data buffer for parallelized PIPG
  vector_array_t X_, W_, V_, U_;
  vector_array_t XNew_, UNew_, WNew_;
};

}  // namespace bolza
