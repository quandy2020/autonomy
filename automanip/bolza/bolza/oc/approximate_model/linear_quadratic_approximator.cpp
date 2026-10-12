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

#include <iostream>

#include "bolza/oc/approximate_model/linear_quadratic_approximator.hpp"

#include <bolza/core/misc/linear_algebra.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ApproximateIntermediateLQ(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                               const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  preComputation.request(request, time, state, input);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = input.rows();

  // Dynamics
  modelData.dynamicsCovariance = problem.dynamicsPtr->dynamicsCovariance(time, state, input);
  modelData.dynamics = problem.dynamicsPtr->linearApproximation(time, state, input, preComputation);
  modelData.dynamicsBias.setZero(modelData.dynamics.dfdx.rows());

  // Cost
  modelData.cost = bolza::ApproximateCost(problem, time, state, input);

  // Equality constraints
  modelData.stateEqConstraint = problem.stateEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);
  modelData.stateInputEqConstraint = problem.equalityConstraintPtr->getLinearApproximation(time, state, input, preComputation);

  // Lagrangians
  if (!problem.stateEqualityLagrangianPtr->empty()) {
    auto approx = problem.stateEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.stateInequalityLagrangianPtr->empty()) {
    auto approx = problem.stateInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.equalityLagrangianPtr->empty()) {
    modelData.cost +=
        problem.equalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputEq, preComputation);
  }
  if (!problem.inequalityLagrangianPtr->empty()) {
    modelData.cost +=
        problem.inequalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputIneq, preComputation);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ApproximatePreJumpLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                          const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  preComputation.requestPreJump(request, time, state);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = 0;

  // Jump map
  modelData.dynamics = problem.dynamicsPtr->jumpMapLinearApproximation(time, state, preComputation);
  modelData.dynamicsBias.setZero(modelData.dynamics.dfdx.rows());

  // Pre-jump cost
  modelData.cost = ApproximateEventCost(problem, time, state);

  // state equality constraint
  modelData.stateEqConstraint = problem.preJumpEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);

  // Lagrangians
  if (!problem.preJumpEqualityLagrangianPtr->empty()) {
    auto approx = problem.preJumpEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.preJumpInequalityLagrangianPtr->empty()) {
    auto approx = problem.preJumpInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ApproximateFinalLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                        const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  preComputation.requestFinal(request, time, state);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = 0;
  modelData.dynamicsBias = vector_t();

  // Dynamics
  modelData.dynamics = VectorFunctionLinearApproximation();

  // state equality constraint
  modelData.stateEqConstraint = problem.finalEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);

  // Final cost
  modelData.cost = ApproximateFinalCost(problem, time, state);

  // Lagrangians
  if (!problem.finalEqualityLagrangianPtr->empty()) {
    auto approx = problem.finalEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.finalInequalityLagrangianPtr->empty()) {
    auto approx = problem.finalInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ComputeCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state, const vector_t& input) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // Compute and sum all costs
  auto cost = problem.costPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.softConstraintPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.stateCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.stateSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ApproximateCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                                     const vector_t& input) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // get the state-input cost approximations
  auto cost = problem.costPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);

  if (!problem.softConstraintPtr->empty()) {
    cost += problem.softConstraintPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);
  }

  // get the state only cost approximations
  if (!problem.stateCostPtr->empty()) {
    auto stateCost = problem.stateCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  if (!problem.stateSoftConstraintPtr->empty()) {
    auto stateCost = problem.stateSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ComputeEventCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.preJumpSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ApproximateEventCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.preJumpSoftConstraintPtr->empty()) {
    cost += problem.preJumpSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ComputeFinalCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.finalSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ApproximateFinalCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.finalSoftConstraintPtr->empty()) {
    cost += problem.finalSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputeIntermediateMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                                   vector_t&& dynamicsViolation) {
  auto& preComputation = *problem.preComputationPtr;

  Metrics metrics;

  // Cost
  metrics.cost = ComputeCost(problem, time, state, input);

  // Dynamics violation
  metrics.dynamicsViolation = std::move(dynamicsViolation);

  // Equality constraints
  if (!problem.stateEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint = problem.stateEqualityConstraintPtr->getValue(time, state, preComputation);
  }
  if (!problem.equalityConstraintPtr->empty()) {
    metrics.stateInputEqConstraint = problem.equalityConstraintPtr->getValue(time, state, input, preComputation);
  }

  // Inequality constraints
  if (!problem.stateInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint = problem.stateInequalityConstraintPtr->getValue(time, state, preComputation);
  }
  if (!problem.inequalityConstraintPtr->empty()) {
    metrics.stateInputIneqConstraint = problem.inequalityConstraintPtr->getValue(time, state, input, preComputation);
  }

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputeIntermediateMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                                   const MultiplierCollection& multipliers, vector_t&& dynamicsViolation) {
  auto& preComputation = *problem.preComputationPtr;

  // cost, dynamics violation, equlaity constraints, inequlaity constraints
  auto metrics = ComputeIntermediateMetrics(problem, time, state, input, std::move(dynamicsViolation));

  // Equality Lagrangians
  metrics.stateEqLagrangian = problem.stateEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  metrics.stateInputEqLagrangian = problem.equalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputEq, preComputation);

  // Inequality Lagrangians
  metrics.stateIneqLagrangian = problem.stateInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);
  metrics.stateInputIneqLagrangian =
      problem.inequalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputIneq, preComputation);

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputePreJumpMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, vector_t&& dynamicsViolation) {
  auto& preComputation = *problem.preComputationPtr;

  Metrics metrics;

  // Cost
  metrics.cost = ComputeEventCost(problem, time, state);

  // Dynamics violation
  metrics.dynamicsViolation = std::move(dynamicsViolation);

  // Equality constraint
  if (!problem.preJumpEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint = problem.preJumpEqualityConstraintPtr->getValue(time, state, preComputation);
  }

  // Inequality constraint
  if (!problem.preJumpInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint = problem.preJumpInequalityConstraintPtr->getValue(time, state, preComputation);
  }

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputePreJumpMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                              const MultiplierCollection& multipliers, vector_t&& dynamicsViolation) {
  auto& preComputation = *problem.preComputationPtr;

  // cost, dynamics violation, equlaity constraints, inequlaity constraints
  auto metrics = ComputePreJumpMetrics(problem, time, state, std::move(dynamicsViolation));

  // Equality Lagrangians
  metrics.stateEqLagrangian = problem.preJumpEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);

  // Inequality Lagrangians
  metrics.stateIneqLagrangian = problem.preJumpInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputeFinalMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state) {
  auto& preComputation = *problem.preComputationPtr;

  Metrics metrics;

  // Cost
  metrics.cost = ComputeFinalCost(problem, time, state);

  // Dynamics violation
  // metrics.dynamicsViolation = vector_t();

  // Equality constraint
  if (!problem.finalEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint = problem.finalEqualityConstraintPtr->getValue(time, state, preComputation);
  }

  // Inequality constraint
  if (!problem.finalInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint = problem.finalInequalityConstraintPtr->getValue(time, state, preComputation);
  }

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics ComputeFinalMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                            const MultiplierCollection& multipliers) {
  auto& preComputation = *problem.preComputationPtr;

  // cost, equlaity constraints, inequlaity constraints
  auto metrics = ComputeFinalMetrics(problem, time, state);

  // Equality Lagrangians
  metrics.stateEqLagrangian = problem.finalEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);

  // Inequality Lagrangians
  metrics.stateIneqLagrangian = problem.finalInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);

  return metrics;
}

}  // namespace bolza
