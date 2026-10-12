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

#include "bolza/oc/oc_problem/loopshaping_optimal_control_problem.hpp"

#include <bolza/core/loopshaping/loopshaping.hpp>

namespace bolza {
namespace LoopshapingOptimalControlProblem {

OptimalControlProblem Create(const OptimalControlProblem& problem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  OptimalControlProblem augmentedProblem;

  // Dynamics
  augmentedProblem.dynamicsPtr = LoopshapingDynamics::Create(*problem.dynamicsPtr, loopshapingDefinition);

  // Equality constraints
  augmentedProblem.equalityConstraintPtr = LoopshapingConstraint::Create(*problem.equalityConstraintPtr, loopshapingDefinition);
  augmentedProblem.stateEqualityConstraintPtr = LoopshapingConstraint::Create(*problem.stateEqualityConstraintPtr, loopshapingDefinition);
  augmentedProblem.preJumpEqualityConstraintPtr =
      LoopshapingConstraint::Create(*problem.preJumpEqualityConstraintPtr, loopshapingDefinition);
  augmentedProblem.finalEqualityConstraintPtr = LoopshapingConstraint::Create(*problem.finalEqualityConstraintPtr, loopshapingDefinition);

  // Inequality constraints
  augmentedProblem.inequalityConstraintPtr = LoopshapingConstraint::Create(*problem.inequalityConstraintPtr, loopshapingDefinition);
  augmentedProblem.stateInequalityConstraintPtr =
      LoopshapingConstraint::Create(*problem.stateInequalityConstraintPtr, loopshapingDefinition);
  augmentedProblem.preJumpInequalityConstraintPtr =
      LoopshapingConstraint::Create(*problem.preJumpInequalityConstraintPtr, loopshapingDefinition);
  augmentedProblem.finalInequalityConstraintPtr =
      LoopshapingConstraint::Create(*problem.finalInequalityConstraintPtr, loopshapingDefinition);

  // Lagrangians
  augmentedProblem.equalityLagrangianPtr = LoopshapingAugmentedLagrangian::Create(*problem.equalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.stateEqualityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.stateEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.inequalityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.inequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.stateInequalityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.stateInequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.preJumpEqualityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.preJumpEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.preJumpInequalityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.preJumpInequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.finalEqualityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.finalEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.finalInequalityLagrangianPtr =
      LoopshapingAugmentedLagrangian::Create(*problem.finalInequalityLagrangianPtr, loopshapingDefinition);

  // Soft constraints
  augmentedProblem.softConstraintPtr = LoopshapingSoftConstraint::Create(*problem.softConstraintPtr, loopshapingDefinition);
  augmentedProblem.stateSoftConstraintPtr = LoopshapingSoftConstraint::Create(*problem.stateSoftConstraintPtr, loopshapingDefinition);
  augmentedProblem.preJumpSoftConstraintPtr = LoopshapingSoftConstraint::Create(*problem.preJumpSoftConstraintPtr, loopshapingDefinition);
  augmentedProblem.finalSoftConstraintPtr = LoopshapingSoftConstraint::Create(*problem.finalSoftConstraintPtr, loopshapingDefinition);

  // Cost
  augmentedProblem.costPtr = LoopshapingCost::Create(*problem.costPtr, loopshapingDefinition);
  augmentedProblem.stateCostPtr = LoopshapingCost::Create(*problem.stateCostPtr, loopshapingDefinition);
  augmentedProblem.preJumpCostPtr = LoopshapingCost::Create(*problem.preJumpCostPtr, loopshapingDefinition);
  augmentedProblem.finalCostPtr = LoopshapingCost::Create(*problem.finalCostPtr, loopshapingDefinition);

  // Pre-computation
  augmentedProblem.preComputationPtr.reset(new LoopshapingPreComputation(*problem.preComputationPtr, loopshapingDefinition));

  /** The cost desired trajectories (will be substitute by ReferenceManager) */
  augmentedProblem.targetTrajectoriesPtr = problem.targetTrajectoriesPtr;

  return augmentedProblem;
}

}  // namespace LoopshapingOptimalControlProblem
}  // namespace bolza
