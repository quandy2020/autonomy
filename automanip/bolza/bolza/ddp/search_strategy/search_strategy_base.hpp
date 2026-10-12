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
 * @file search_strategy_base.hpp
 * @brief This class is an interface class for search strategies such as line-search, trust-region.
 */


#include <functional>
#include <utility>
#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/model_data.hpp>
#include <bolza/core/reference/mode_schedule.hpp>
#include <bolza/oc/oc_data/dual_solution.hpp>
#include <bolza/oc/oc_data/performance_index.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/oc_data/problem_metrics.hpp>

#include "bolza/ddp/search_strategy/strategy_settings.hpp"

namespace bolza {

// forward declaration
namespace search_strategy {
struct Solution;
struct SolutionRef;
}  // namespace search_strategy

/**
 * This class is an interface class for search strategies such as line-search, trust-region.
 */
class SearchStrategyBase {
 public:
  /**
   * Constructor.
   * @param [in] baseSettings: The basic settings for the search strategy algorithms.
   */
  explicit SearchStrategyBase(search_strategy::Settings baseSettings) : baseSettings_(std::move(baseSettings)) {}

  /**
   * @brief Destructor.
   */
  virtual ~SearchStrategyBase() = default;
  /**
   * @brief Constructs a SearchStrategyBase.
   */
  SearchStrategyBase(const SearchStrategyBase&) = delete;
  SearchStrategyBase& operator=(const SearchStrategyBase&) = delete;

  /**
   * Resets the class to its state after construction.
   */
  virtual void Reset() = 0;

    /**
   * @brief.
   *
   * Finds the optimal trajectories, controller, and performance index based on the given controller and its increment.
   *
   * /
   * @param [in] timePeriod: Initial and final times pair.
   * @param [in] initState: Initial state
   * @param [in] expectedCost: The expected cost based on the LQ model optimization.
   * @param [in] unoptimizedController: The unoptimized controller which search will be performed.
   * @param [in] dualSolution: The dual solution.
   * @param [in] modeSchedule: Mode Schedule.
   * @param [out] solution: Output of search (primalSolution, performanceIndex, problemMetrics, avgTimeStep)
   * @return whether the search was successful or failed.
   */
  virtual bool Run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState, const scalar_t expectedCost,
                   const LinearController& unoptimizedController, const DualSolution& dualSolution, const ModeSchedule& modeSchedule,
                   search_strategy::SolutionRef solution) = 0;

  /**
   * Checks convergence of the main loop of DDP.
   *
   * @param [in] unreliableControllerIncrement: True if the controller is designed based on an unreliable LQ approximation such as
   * operating trajectories
   * @param [in] previousPerformanceIndex: The previous iteration's PerformanceIndex.
   * @param [in] currentPerformanceIndex: The current iteration's PerformanceIndex.
   * @return A pair of (isOptimizationConverged, infoString)
   */
  virtual std::pair<bool, std::string> CheckConvergence(bool unreliableControllerIncrement,
                                                        const PerformanceIndex& previousPerformanceIndex,
                                                        const PerformanceIndex& currentPerformanceIndex) const = 0;

  /**
   * Computes the Riccati modification based on the strategy.
   *
   * @param [in] projectedModelData: The projected data model
   * @param [out] deltaQm: The Riccati modifier to cost 2nd derivative w.r.t. state.
   * @param [out] deltaGv: The Riccati modifier to cost derivative w.r.t. input.
   * @param [out] deltaGm: The Riccati modifier to cost input-state derivative.
   */
  virtual void ComputeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                          matrix_t& deltaGm) const = 0;

  /**
   * Augments the Hessian of Hamiltonian based on the strategy.
   *
   * @param [in] modelData: The model data.
   * @param [in] Hm: The Hessian of Hamiltonian that should be augmented.
   * @return The augmented Hamiltonian's Hessian.
   */
  virtual matrix_t AugmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const = 0;

 protected:
  const search_strategy::Settings baseSettings_;
};

}  // namespace bolza

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace bolza {
namespace search_strategy {

struct Solution {
  scalar_t avgTimeStep;
  DualSolution dualSolution;
  PrimalSolution primalSolution;
  ProblemMetrics problemMetrics;
  PerformanceIndex performanceIndex;
};

struct SolutionRef {
  /**
   * @brief Constructs a SolutionRef.
   * @param [in] performanceIndex: Performance Index.
   */
  SolutionRef(Solution& s)
      /**
       * @brief Avg Time Step.
       * @param [in] performanceIndex: Performance Index.
       * @return Result value.
       */
      : avgTimeStep(s.avgTimeStep),
        dualSolution(s.dualSolution),
        primalSolution(s.primalSolution),
        problemMetrics(s.problemMetrics),
        performanceIndex(s.performanceIndex) {}

  /**
   * @brief Constructs a SolutionRef.
   * @param [in,out] avgTimeStepArg: Avg Time Step Arg.
   * @param [in,out] dualSolutionArg: Dual Solution Arg.
   * @param [in,out] primalSolutionArg: Primal Solution Arg.
   * @param [in,out] problemMetricsArg: Problem Metrics Arg.
   * @param [in] performanceIndexArg: Performance Index Arg.
   */
  SolutionRef(scalar_t& avgTimeStepArg, DualSolution& dualSolutionArg, PrimalSolution& primalSolutionArg, ProblemMetrics& problemMetricsArg,
              PerformanceIndex& performanceIndexArg)
      /**
       * @brief Avg Time Step.
       * @param [in] performanceIndexArg: Performance Index Arg.
       * @return Result value.
       */
      : avgTimeStep(avgTimeStepArg),
        dualSolution(dualSolutionArg),
        primalSolution(primalSolutionArg),
        problemMetrics(problemMetricsArg),
        performanceIndex(performanceIndexArg) {}

  scalar_t& avgTimeStep;
  DualSolution& dualSolution;
  PrimalSolution& primalSolution;
  ProblemMetrics& problemMetrics;
  PerformanceIndex& performanceIndex;
};

/**
 * @brief Swap.
 * @param [in] lhs: Lhs.
 * @param [in] rhs: Rhs.
 */
inline void swap(SolutionRef lhs, SolutionRef rhs) {
  /**
   * @brief Swap.
   * @param [in] avgTimeStep: Avg Time Step.
   * @param [in] avgTimeStep: Avg Time Step.
   * @return Result value.
   */
  std::swap(lhs.avgTimeStep, rhs.avgTimeStep);
  /**
   * @brief Swap.
   * @param [in] dualSolution: Dual Solution.
   * @return Result value.
   */
  lhs.dualSolution.swap(rhs.dualSolution);
  /**
   * @brief Swap.
   * @param [in] primalSolution: Primal Solution.
   * @return Result value.
   */
  lhs.primalSolution.swap(rhs.primalSolution);
  /**
   * @brief Swap.
   * @param [in] problemMetrics: Problem Metrics.
   * @return Result value.
   */
  lhs.problemMetrics.swap(rhs.problemMetrics);
  /**
   * @brief Swap.
   * @param [in] performanceIndex: Performance Index.
   * @param [in] performanceIndex: Performance Index.
   * @return Result value.
   */
  bolza::swap(lhs.performanceIndex, rhs.performanceIndex);
}

}  // namespace search_strategy
}  // namespace bolza
