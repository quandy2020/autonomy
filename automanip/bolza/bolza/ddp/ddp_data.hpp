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
 * @file ddp_data.hpp
 * @brief Primal data container.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/model_data.hpp>
#include <bolza/oc/oc_data/dual_solution.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/oc_data/problem_metrics.hpp>

#include "bolza/ddp/riccati_equations/riccati_modification.hpp"

namespace bolza {

/**
 * Primal data container
 *
 * The design philosophy behind is to keep all member variables consistent. All (time, post, .., modelDataEventTime)
 * trajectories should be the rollout result of the controller
 *
 * There is one exception that breaks the consistency. When using an external controller to initialize the controller, it is obvious that
 * the rest of member variables are not the result of the controller. But they will be cleared and populated when runInit is called.
 */
struct PrimalDataContainer {
  // Primal solution
  PrimalSolution primalSolution;
  // cost, soft constraints and constraints values of the rollout
  ProblemMetrics problemMetrics;
  // final time model data
  ModelData modelDataFinalTime;
  // event times model data
  std::vector<ModelData> modelDataEventTimes;
  // intermediate model data trajectory
  std::vector<ModelData> modelDataTrajectory;

  /**
   * @brief Swap.
   * @param [in,out] other: Other.
   */
  void swap(PrimalDataContainer& other) {
    /**
     * @brief Swap.
     * @param [in] primalSolution: Primal Solution.
     * @return Result value.
     */
    primalSolution.swap(other.primalSolution);
    /**
     * @brief Swap.
     * @param [in] problemMetrics: Problem Metrics.
     * @return Result value.
     */
    problemMetrics.swap(other.problemMetrics);
    /**
     * @brief Swap.
     * @param [in] modelDataFinalTime: Model Data Final Time.
     * @param [in] modelDataFinalTime: Model Data Final Time.
     * @return Result value.
     */
    std::swap(modelDataFinalTime, other.modelDataFinalTime);
    /**
     * @brief Swap.
     * @param [in] modelDataEventTimes: Model Data Event Times.
     * @return Result value.
     */
    modelDataEventTimes.swap(other.modelDataEventTimes);
    /**
     * @brief Swap.
     * @param [in] modelDataTrajectory: Model Data Trajectory.
     * @return Result value.
     */
    modelDataTrajectory.swap(other.modelDataTrajectory);
  }

  /**
   * @brief Clear.
   */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    primalSolution.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    problemMetrics.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    modelDataEventTimes.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    modelDataTrajectory.clear();
  }
};

/**
 * Dual data container
 *
 * The design philosophy behind is to keep all member variables consistent. valueFunctionTrajectory is the direct result of
 * (projectedModelData,riccatiModification) trajectories.
 *
 */
struct DualDataContainer {
  // Dual solution
  DualSolution dualSolution;
  // projected model data trajectory
  std::vector<ModelData> projectedModelDataTrajectory;
  // Riccati modification
  std::vector<riccati_modification::Data> riccatiModificationTrajectory;
  // Riccati solution coefficients
  std::vector<ScalarFunctionQuadraticApproximation> valueFunctionTrajectory;

  /**
   * @brief Swap.
   * @param [in,out] other: Other.
   */
  void swap(DualDataContainer& other) {
    /**
     * @brief Swap.
     * @param [in] dualSolution: Dual Solution.
     * @return Result value.
     */
    dualSolution.swap(other.dualSolution);
    /**
     * @brief Swap.
     * @param [in] projectedModelDataTrajectory: Projected Model Data Trajectory.
     * @return Result value.
     */
    projectedModelDataTrajectory.swap(other.projectedModelDataTrajectory);
    /**
     * @brief Swap.
     * @param [in] riccatiModificationTrajectory: Riccati Modification Trajectory.
     * @return Result value.
     */
    riccatiModificationTrajectory.swap(other.riccatiModificationTrajectory);
    /**
     * @brief Swap.
     * @param [in] valueFunctionTrajectory: Value Function Trajectory.
     * @return Result value.
     */
    valueFunctionTrajectory.swap(other.valueFunctionTrajectory);
  }

  /**
   * @brief Clear.
   */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    dualSolution.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    projectedModelDataTrajectory.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    riccatiModificationTrajectory.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    valueFunctionTrajectory.clear();
  }
};

}  // namespace bolza
