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
 * @file optimal_control_problem_helper_function.hpp
 * @brief Initializes the dual solution based on the cached dual solution. It will use interpolation if cachedDualSolution has any component.
 */


#include <functional>

#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/multiplier.hpp>

#include "bolza/oc/oc_data/dual_solution.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/oc_data/problem_metrics.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"

namespace bolza {

/**
 * Initializes the dual solution based on the cached dual solution. It will use interpolation if cachedDualSolution has any component
 * in the same mode otherwise it will use the Lagrangian initialization method of ocp.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] primalSolution: The primal solution.
 * @param [in] cachedDualSolution: The cached dual solution which will be used for interpolation.
 * @param [out] dualSolution: The initialized dual solution.
 */
void InitializeDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const DualSolution& cachedDualSolution,
                            DualSolution& dualSolution);

/**
 * Initializes final MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Final time.
 * @param [out] multiplierCollection: The initialized final MultiplierCollection.
 */
void InitializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/**
 * Initializes pre-jump MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Pre-jump time.
 * @param [out] multiplierCollection: The initialized pre-jump MultiplierCollection.
 */
void InitializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/**
 * Initializes intermediate MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Intermediate time.
 * @param [out] multiplierCollection: The initialized intermediate MultiplierCollection.
 */
void InitializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time,
                                                MultiplierCollection& multiplierCollection);

/**
 * Updates in-place the dual solution based on its current solution and state-input values using the Lagrangian update method in ocp.
 * Moreover it also updates the penalties of ProblemMetrics based on the update of dual solution.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] primalSolution: The primal solution.
 * @param [in,out] problemMetrics: The problem metric. Its penalties will be updated based on the update of dualSolution.
 * @param [out] dualSolution: The updated dual solution.
 */
void UpdateDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, ProblemMetrics& problemMetrics,
                        DualSolutionRef dualSolution);

/**
 * Updates in-place final MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Final time.
 * @param [in] state: Final state.
 * @param [in,out] metrics: The final Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers: The updated final MultiplierCollection.
 */
void UpdateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                     MultiplierCollection& multipliers);

/**
 * Updates in-place pre-jump MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Pre-jump time.
 * @param [in] state: Pre-jump state.
 * @param [in,out] metrics: The pre-jump Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers: The updated pre-jump MultiplierCollection.
 */
void UpdatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                       MultiplierCollection& multipliers);

/**
 * Updates in-place intermediate MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] time: Intermediate time.
 * @param [in] state: Intermediate state.
 * @param [in] input: Intermediate input.
 * @param [in,out] metrics: The intermediate Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers: The updated Intermediate MultiplierCollection.
 */
void UpdateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            Metrics& metrics, MultiplierCollection& multipliers);

/**
 * Extracts a requested final constraint term vector from the input Metrics.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metrics: Metrics.
 * @return A const pointer to the constraint term value.
 */
const vector_t* ExtractFinalTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const Metrics& metrics);

/**
 * Extracts a requested final term LagrangianMetrics from the input Metrics.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metrics: Metrics.
 * @return A const pointer to the term LagrangianMetrics.
 */
const LagrangianMetrics* ExtractFinalTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                                           const Metrics& metrics);

/**
 * Extracts an array of the requested pre-jump constraint term from the input Metrics array.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metricsArray: Metrics array.
 * @param [out] constraintArray: Array of const references to constraint term values.
 * @return True if the term found in the pre-jump collection.
 */
bool ExtractPreJumpTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsArray,
                                  std::vector<std::reference_wrapper<const vector_t>>& constraintArray);

/**
 * Extracts a requested pre-jump term LagrangianMetrics array from the input Metrics array.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metricsArray: Metrics array.
 * @param [out] lagrangianMetricsArray: Array of const references to term LagrangianMetrics.
 * @return True if the term found in the pre-jump collection.
 */
bool ExtractPreJumpTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                         const std::vector<Metrics>& metricsArray,
                                         std::vector<LagrangianMetricsConstRef>& lagrangianMetricsArray);

/**
 * Extracts a trajectory of the requested intermediate constraint term from the input Metrics trajectory.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metricsTraj: Metrics trajectory.
 * @param [out] constraintTraj: Trajectory of const references to constraint term values.
 * @return True if the term found in the intermediate collection.
 */
bool ExtractIntermediateTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsTraj,
                                       std::vector<std::reference_wrapper<const vector_t>>& constraintTraj);

/**
 * Extracts a requested intermediate term LagrangianMetrics trajectory from the input Metrics trajectory.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] metricsTraj: Metrics trajectory.
 * @param [out] lagrangianMetricsTraj: Trajectory of const references to term LagrangianMetrics.
 * @return True if the term found in the intermediate collection.
 */
bool ExtractIntermediateTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                              const std::vector<Metrics>& metricsTraj,
                                              std::vector<LagrangianMetricsConstRef>& lagrangianMetricsTraj);

/**
 * Extracts a requested final Term Multiplier from the input MultiplierCollection.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] multiplierColl: MultiplierCollection.
 * @param [out] multiplier: A const reference to the term Multiplier.
 * @return True if the term found in the final collection.
 */
const Multiplier* ExtractFinalTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                             const MultiplierCollection& multiplierColl);

/**
 * Extracts a requested pre-jump Term Multiplier array from the input MultiplierCollection array.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] multiplierCollArray: MultiplierCollection array.
 * @param [out] multiplierArray: Array of const references to term Multiplier.
 * @return True if the term found in the pre-jump collection.
 */
bool ExtractPreJumpTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                  const std::vector<MultiplierCollection>& multiplierCollArray,
                                  std::vector<MultiplierConstRef>& multiplierArray);

/**
 * Extracts a requested intermediate Term Multiplier trajectory from the input MultiplierCollection trajectory.
 * @param [in] ocp: A const reference to the optimal control problem.
 * @param [in] name: The name of the term.
 * @param [in] multiplierCollTraj: MultiplierCollection trajectory.
 * @param [out] multiplierTrajectory: Trajectory of const references to term Multiplier.
 * @return True if the term found in the intermediate collection.
 */
bool ExtractIntermediateTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                       const std::vector<MultiplierCollection>& multiplierCollTraj,
                                       std::vector<MultiplierConstRef>& multiplierTrajectory);

}  // namespace bolza
