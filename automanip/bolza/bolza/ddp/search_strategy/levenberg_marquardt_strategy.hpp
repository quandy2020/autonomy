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
 * @file levenberg_marquardt_strategy.hpp
 * @brief Levenberg Marquardt strategy: The class computes the nominal controller and the nominal trajectories.
 */


#include <functional>
#include <utility>
#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/rollout/rollout_base.hpp>

#include "bolza/ddp/search_strategy/search_strategy_base.hpp"
#include "bolza/ddp/search_strategy/strategy_settings.hpp"

namespace bolza {

/**
 * Levenberg Marquardt strategy: The class computes the nominal controller and the nominal trajectories
 * as well the corresponding performance indices.
 * reference: Tassa et al., Synthesis and stabilization of complex behaviors through online trajectory optimization.
 */
class LevenbergMarquardtStrategy final : public SearchStrategyBase {
 public:
    /**
   * @brief.
   *
   * Constructor.
   *
   * /
   * @param [in] baseSettings: The basic settings for the search strategy algorithms.
   * @param [in] settings: The Levenberg Marquardt settings.
   * @param [in,out] rolloutRefStock: Rollout Ref Stock.
   * @param [in] optimalControlProblemRef: A reference to the optimal control problem.
   * @param [in] meritFunc: the merit function which gets the PerformanceIndex and returns the merit function value.
   */
  LevenbergMarquardtStrategy(search_strategy::Settings baseSettings, levenberg_marquardt::Settings settings, RolloutBase& rolloutRefStock,
                             OptimalControlProblem& optimalControlProblemRef, std::function<scalar_t(const PerformanceIndex&)> meritFunc);

  /**
   * @brief Destructor.
   */
  ~LevenbergMarquardtStrategy() override = default;
  /**
   * @brief Constructs a LevenbergMarquardtStrategy.
   */
  LevenbergMarquardtStrategy(const LevenbergMarquardtStrategy&) = delete;
  LevenbergMarquardtStrategy& operator=(const LevenbergMarquardtStrategy&) = delete;

  /**
   * @brief Reset.
   */
  void Reset() override;

  /**
   * @brief Run.
   * @param [in] timePeriod: Time Period.
   * @param [in] initState: Init State.
   * @param [in] expectedCost: Expected Cost.
   * @param [in] unoptimizedController: Unoptimized Controller.
   * @param [in] dualSolution: Dual Solution.
   * @param [in] modeSchedule: Mode Schedule.
   * @param [in] solution: Solution.
   * @return True on success.
   */
  bool Run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState, const scalar_t expectedCost,
           const LinearController& unoptimizedController, const DualSolution& dualSolution, const ModeSchedule& modeSchedule,
           search_strategy::SolutionRef solution) override;

  /**
   * @brief Check Convergence.
   * @param [in] unreliableControllerIncrement: Unreliable Controller Increment.
   * @param [in] previousPerformanceIndex: Previous Performance Index.
   * @param [in] currentPerformanceIndex: Current Performance Index.
   * @return Result value.
   */
  std::pair<bool, std::string> CheckConvergence(bool unreliableControllerIncrement, const PerformanceIndex& previousPerformanceIndex,
                                                const PerformanceIndex& currentPerformanceIndex) const override;

  /**
   * @brief Compute Riccati Modification.
   * @param [in] projectedModelData: Projected Model Data.
   * @param [in,out] deltaQm: Delta Qm.
   * @param [in,out] deltaGv: Delta Gv.
   * @param [in,out] deltaGm: Delta Gm.
   */
  void ComputeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                  matrix_t& deltaGm) const override;

  /**
   * @brief Augment Hamiltonian Hessian.
   * @param [in] modelData: Model Data.
   * @param [in] Hm: Hm.
   * @return Result value.
   */
  matrix_t AugmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const override;

 private:
    /**
   * @brief computes the ratio between actual reduction and predicted reduction 
   * @param [in] actualReduction: Actual Reduction.
   * @param [in] expectedReduction: Expected Reduction.
   * @return Result value.
   */
  scalar_t ReductionToPredictedReduction(const scalar_t actualReduction, const scalar_t expectedReduction) const {
    if (std::abs(actualReduction) < baseSettings_.minRelCost || expectedReduction <= baseSettings_.minRelCost) {
      return 1.0;
    } else if (actualReduction < 0.0) {
      return 0.0;
    } else {
      return actualReduction / expectedReduction;
    }
  }

  // Levenberg-Marquardt
  struct LevenbergMarquardtModule {
    scalar_t riccatiMultiple = 0.0;               // the Riccati multiple for Tikhonov regularization.
    scalar_t riccatiMultipleAdaptiveRatio = 1.0;  // the adaptive ratio of geometric progression for Riccati multiple.
    size_t numSuccessiveRejections = 0;           // the number of successive rejections of solution.
  };

  const levenberg_marquardt::Settings settings_;
  LevenbergMarquardtModule lmModule_;

  RolloutBase& rolloutRef_;
  OptimalControlProblem& optimalControlProblemRef_;
  std::function<scalar_t(PerformanceIndex)> meritFunc_;

  DualSolution tempDualSolution_;
};

}  // namespace bolza
