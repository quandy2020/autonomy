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
 * @file ilqr.hpp
 * @brief This class is an interface class for the single-thread and multi-thread ILQR.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>

#include "gauss_newton_ddp.hpp"
#include "riccati_equations/discrete_time_riccati_equations.hpp"

namespace bolza {

/**
 * This class is an interface class for the single-thread and multi-thread ILQR.
 */
class ILQR : public GaussNewtonDDP {
 public:
  /**
   * Constructor
   *
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  ILQR(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
       const Initializer& initializer);

  /**
   * Default destructor.
   */
  ~ILQR() override = default;

 protected:
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
   * @brief Calculate Controller Worker.
   * @param [in] timeIndex: Time Index.
   * @param [in] primalData: Primal Data.
   * @param [in] dualData: Dual Data.
   * @param [in,out] dstController: Dst Controller.
   */
  void CalculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                 LinearController& dstController) override;

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
   * Calculates the discrete-time LQ approximation from the continuous-time LQ approximation.
   *
   * @param [in] system: system dynamic.
   * @param [in] time: time t_k.
   * @param [in] state: state x_k.
   * @param [in] input: input u_k.
   * @param [in] timeStep: Time step between the x_{k} and x_{k+1}.
   * @param [in] continuousTimeModelData: continuous time model data.
   * @param [out] modelData: Discretized mode data.
   */
  void DiscreteLQWorker(SystemDynamicsBase& system, scalar_t time, const vector_t& state, const vector_t& input, scalar_t timeStep,
                        const ModelData& continuousTimeModelData, ModelData& modelData);

  /****************
   *** Variables **
   ****************/
  matrix_array_t projectedKmTrajectoryStock_;  // projected feedback
  vector_array_t projectedLvTrajectoryStock_;  // projected feedforward

  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<std::unique_ptr<DiscreteTimeRiccatiEquations>> riccatiEquationsPtrStock_;
};

}  // namespace bolza
