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
 * @file slq.hpp
 * @brief This class is an interface class for the single-thread and multi-thread SLQ.
 */


#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/integration/system_event_handler.hpp>

#include "bolza/ddp/gauss_newton_ddp.hpp"
#include "bolza/ddp/riccati_equations/continuous_time_riccati_equations.hpp"

namespace bolza {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 */
class SLQ final : public GaussNewtonDDP {
 public:
  /**
   * Constructor
   *
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  SLQ(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
      const Initializer& initializer);

  /**
   * Default destructor.
   */
  ~SLQ() override = default;

 protected:
  /**
   * @brief Compute Hamiltonian Hessian.
   * @param [in] modelData: Model Data.
   * @param [in] Sm: Sm.
   * @return Result value.
   */
  matrix_t ComputeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const override;

  /**
   * @brief Approximate Intermediate LQ.
   * @param [in] dualSolution: Dual Solution.
   * @param [in,out] primalData: Primal Data.
   */
  void ApproximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) override;

  /**
   * @brief Calculate Controller Worker.
   * @param [in] timeIndex: Time Index.
   * @param [in] primalData: Primal Data.
   * @param [in] dualData: Dual Data.
   * @param [in,out] dstController: Dst Controller.
   */
  void CalculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                 LinearController& dstController) override;

  /**
   * @brief Solve Sequential Riccati Equations.
   * @param [in] finalValueFunction: Final Value Function.
   * @return Result value.
   */
  scalar_t SolveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) override;

  /**
   * @brief Riccati Equations Worker.
   * @param [in] workerIndex: Worker Index.
   * @param [in] partitionInterval: Partition Interval.
   * @param [in] finalValueFunction: Final Value Function.
   */
  void RiccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                              const ScalarFunctionQuadraticApproximation& finalValueFunction) override;

    /**
   * @brief.
   *
   * Integrates the riccati equation and generates the value function at the times set in nominal Time Trajectory.
   *
   * /
   * @param [in] riccatiIntegrator: [in] : Riccati integrator object
   * @param [in] riccatiEquation: [in] : Riccati equation object
   * @param [in] partitionInterval: Partition Interval.
   * @param [in] nominalTimeTrajectory: [in] : time trajectory produced in the forward rollout.
   * @param [in] nominalEventsPastTheEndIndices: [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param [in] allSsFinal: [in] : Final value of the value function.
   * @param [in] SsNormalizedTime: [out] : Time trajectory of the value function.
   * @param [in] SsNormalizedPostEventIndices: [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param [in] allSsTrajectory: [out] : Value function in vector format.
   */
  void IntegrateRiccatiEquationNominalTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                           const std::pair<int, int>& partitionInterval, const scalar_array_t& nominalTimeTrajectory,
                                           const size_array_t& nominalEventsPastTheEndIndices, vector_t allSsFinal,
                                           scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                           vector_array_t& allSsTrajectory);

  /****************
   *** Variables **
   ****************/
  std::vector<std::shared_ptr<ContinuousTimeRiccatiEquations>> riccatiEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase>> riccatiIntegratorPtrStock_;
  vector_array2_t allSsTrajectoryStock_;
  scalar_array2_t SsNormalizedTimeTrajectoryStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
};

}  // namespace bolza
