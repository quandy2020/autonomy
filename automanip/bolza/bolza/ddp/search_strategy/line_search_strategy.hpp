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
 * @file line_search_strategy.hpp
 * @brief Line search strategy: The class computes the nominal controller and the nominal trajectories as well the corresponding performance.
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
 * Line search strategy: The class computes the nominal controller and the nominal trajectories as well the corresponding performance
 * indices. It line-searches on the feedforward parts of the controller and chooses the largest acceptable step-size.
 */
class LineSearchStrategy final : public SearchStrategyBase {
 public:
  /**
   * Constructor.
   *
   * @param [in] baseSettings: The basic settings for the search strategy algorithms.
   * @param [in] settings: The line search settings.
   * @param [in] threadPoolRef: A reference to the thread pool instance.
   * @param [in] rolloutRefStock: An array of references to the rollout.
   * @param [in] optimalControlProblemRef: An array of references to the optimal control problem.
   * @param [in] meritFunc: the merit function which gets the PerformanceIndex and returns the merit function value.
   */
  LineSearchStrategy(search_strategy::Settings baseSettings, line_search::Settings settings, ThreadPool& threadPoolRef,
                     std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock,
                     std::vector<std::reference_wrapper<OptimalControlProblem>> optimalControlProblemRef,
                     std::function<scalar_t(const PerformanceIndex&)> meritFunc);

  /**
   * @brief Destructor.
   */
  ~LineSearchStrategy() override = default;
  /**
   * @brief Constructs a LineSearchStrategy.
   */
  LineSearchStrategy(const LineSearchStrategy&) = delete;
  LineSearchStrategy& operator=(const LineSearchStrategy&) = delete;

  /**
   * @brief Reset.
   */
  void Reset() override {}

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
   * @param [in] Hm: Hm.
   * @return Result value.
   */
  matrix_t AugmentHamiltonianHessian(const ModelData& /*modelData*/, const matrix_t& Hm) const override { return Hm; }

 private:
  struct LineSearchInputRef {
    const std::pair<scalar_t, scalar_t>* timePeriodPtr;
    const vector_t* initStatePtr;
    const LinearController* unoptimizedControllerPtr;
    const DualSolution* dualSolutionPtr;
    const ModeSchedule* modeSchedulePtr;
  };

  /**
   * @brief number of line search iterations (the if statements order is important).
   * @return Result value.
   */
  size_t MaxNumOfSearches() const;

    /**
   * @brief Computes the solution on a thread and a given stepLength  
   * @param [in] taskId: Task Id.
   * @param [in] stepLength: Step Length.
   * @param [in,out] solution: Solution.
   */
  void ComputeSolution(size_t taskId, scalar_t stepLength, search_strategy::Solution& solution);

    /**
   * @brief.
   *
   * Defines line search task on a thread with various learning rates and choose the largest acceptable step-size.
   *
   * The class computes the nominal controller and the nominal trajectories as well the corresponding performance indices.
   *
   * /
   * @param [in] taskId: Task Id.
   */
  void LineSearchTask(const size_t taskId);

    /**
   * @brief Prints to output. 
   * @param [in] text: Text.
   */
  void PrintString(const std::string& text) const;

  const line_search::Settings settings_;
  ThreadPool& threadPoolRef_;
  std::vector<DualSolution> tempDualSolutions_;
  std::vector<search_strategy::Solution> workersSolution_;
  std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock_;
  std::vector<std::reference_wrapper<OptimalControlProblem>> optimalControlProblemRefStock_;
  std::function<scalar_t(PerformanceIndex)> meritFunc_;

  // input
  LineSearchInputRef lineSearchInputRef_;
  // output
  std::atomic<scalar_t> bestStepSize_{0.0};
  search_strategy::SolutionRef* bestSolutionRef_;

  // convergence check
  scalar_t baselineMerit_ = 0.0;                  // the merit of the rollout for zero learning rate
  scalar_t unoptimizedControllerUpdateIS_ = 0.0;  // integral of the squared (IS) norm of the controller update.

  // threading
  std::atomic_size_t nextTaskId_{0};
  std::atomic_size_t alphaExpNext_{0};
  std::vector<bool> alphaProcessed_;
  std::mutex lineSearchResultMutex_;
  mutable std::mutex outputDisplayGuardMutex_;
};

}  // namespace bolza
