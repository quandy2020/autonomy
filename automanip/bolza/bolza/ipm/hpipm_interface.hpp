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
 * @file hpipm_interface.hpp
 * @brief HPIPM interface.
 */



#include <memory>

extern "C" {
#include <hpipm_common.h>
}

#include <bolza/core/types.hpp>
#include <bolza/oc/oc_problem/ocp_size.hpp>

#include "bolza/ipm/hpipm_interface_settings.hpp"

namespace bolza {
namespace ipm {

/**
 * This class implements the interface between Linear Quadratic optimal control problems defined in bolza and the HPIPM solver.
 * If the problem dimensions change, resize needs to be called to re-initialize HPIPM.
 */
class HpipmInterface {
 public:
  using Settings = HpipmSettings;

  /**
   * @brief Construct the Hpipm interface with given size and settings.
   *
   * Can directly call Solve() for a problem with consistent size.
   * @param [in] ocpSize: Ocp Size.
   * @param [in] settings: Settings.
   */
  explicit HpipmInterface(OcpSize ocpSize = OcpSize(), const Settings& settings = Settings());

  /** Destructor */
  ~HpipmInterface();

  /**
   * @brief Resize the problem.
   * @param [in] ocpSize: Ocp Size.
   */
  void resize(OcpSize ocpSize);

  /**
   * @brief Solves a discrete linear-quadratic OCP. Resize to a consistent OcpSize first.
   *
   * The problem should be consistently defined in absolute or delta decision variables in x and u.
   * Constraints are mapped to HPIPM inequalities.
   *
   * @param [in] x0: Initial state (deviation).
   * @param [in,out] dynamics: Linearized approximation of the discrete dynamics.
   * @param [in,out] cost: Quadratic approximation of the cost.
   * @param [in,out] constraints: Linearized constraints (nullable).
   * @param [out] stateTrajectory: Solution state (deviation) trajectory.
   * @param [out] inputTrajectory: Solution input (deviation) trajectory.
   * @param [in] verbose: Print HPIPM iteration statistics if true.
   * @return HPIPM status (SUCCESS, MAX_ITER, MIN_STEP, NAN_SOL, INCONS_EQ).
   */
  hpipm_status Solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                     std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                     vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool verbose = false);

  /**
   * @brief Return the Riccati cost-to-go for the previously solved problem.
   *
   * Extra information about the initial stage is needed to complete calculation.
   *
   * Cost-to-go at a node is: V_k(x) = 0.5 * x' * dfdxx * x + x' * dfdx + f
   *
   * For the moment, the value for f is set to 0.0 because it is expensive to compute and often not needed.
   * @param [in] dynamics0: dynamics at k = 0
   * @param [in] cost0: cost at k = 0
   * @return Sequence of quadratic cost-to-go's.
   */
  std::vector<ScalarFunctionQuadraticApproximation> GetRiccatiCostToGo(const VectorFunctionLinearApproximation& dynamics0,
                                                                       const ScalarFunctionQuadraticApproximation& cost0);

  /**
   * @brief Return the sequence of N feedback matrices for the previously solved problem.
   *
   * Extra information about the initial stage is needed to complete calculation.
   * @param [in] dynamics0: dynamics at k = 0
   * @param [in] cost0: cost at k = 0
   * @return Sequence of feedback matrices K of the optimal solution u = K x + k
   */
  matrix_array_t GetRiccatiFeedback(const VectorFunctionLinearApproximation& dynamics0, const ScalarFunctionQuadraticApproximation& cost0);

  /**
   * @brief Return the sequence of N feedforward input vectors for the previously solved problem.
   *
   * Extra information about the initial stage is needed to complete calculation.
   * @param [in] dynamics0: dynamics at k = 0
   * @param [in] cost0: cost at k = 0
   * @return Sequence of feedforward vectors k of the optimal solution u = K x + k
   */
  vector_array_t GetRiccatiFeedforward(const VectorFunctionLinearApproximation& dynamics0,
                                       const ScalarFunctionQuadraticApproximation& cost0);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ipm
}  // namespace bolza