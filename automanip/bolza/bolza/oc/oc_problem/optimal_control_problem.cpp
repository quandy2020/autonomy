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

#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem()
    : /* Cost */
      costPtr(new StateInputCostCollection),
      stateCostPtr(new StateCostCollection),
      preJumpCostPtr(new StateCostCollection),
      finalCostPtr(new StateCostCollection),
      /* Soft constraints */
      softConstraintPtr(new StateInputCostCollection),
      stateSoftConstraintPtr(new StateCostCollection),
      preJumpSoftConstraintPtr(new StateCostCollection),
      finalSoftConstraintPtr(new StateCostCollection),
      /* Equality constraints */
      equalityConstraintPtr(new StateInputConstraintCollection),
      stateEqualityConstraintPtr(new StateConstraintCollection),
      preJumpEqualityConstraintPtr(new StateConstraintCollection),
      finalEqualityConstraintPtr(new StateConstraintCollection),
      /* Inequality constraints */
      inequalityConstraintPtr(new StateInputConstraintCollection),
      stateInequalityConstraintPtr(new StateConstraintCollection),
      preJumpInequalityConstraintPtr(new StateConstraintCollection),
      finalInequalityConstraintPtr(new StateConstraintCollection),
      /* Lagrangians */
      equalityLagrangianPtr(new StateInputAugmentedLagrangianCollection),
      stateEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      inequalityLagrangianPtr(new StateInputAugmentedLagrangianCollection),
      stateInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      preJumpEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      preJumpInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      finalEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      finalInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      /* Misc. */
      preComputationPtr(new PreComputation),
      targetTrajectoriesPtr(nullptr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(const OptimalControlProblem& other)
    : /* Cost */
      costPtr(other.costPtr->Clone()),
      stateCostPtr(other.stateCostPtr->Clone()),
      preJumpCostPtr(other.preJumpCostPtr->Clone()),
      finalCostPtr(other.finalCostPtr->Clone()),
      /* Soft constraints */
      softConstraintPtr(other.softConstraintPtr->Clone()),
      stateSoftConstraintPtr(other.stateSoftConstraintPtr->Clone()),
      preJumpSoftConstraintPtr(other.preJumpSoftConstraintPtr->Clone()),
      finalSoftConstraintPtr(other.finalSoftConstraintPtr->Clone()),
      /* Equality constraints */
      equalityConstraintPtr(other.equalityConstraintPtr->Clone()),
      stateEqualityConstraintPtr(other.stateEqualityConstraintPtr->Clone()),
      preJumpEqualityConstraintPtr(other.preJumpEqualityConstraintPtr->Clone()),
      finalEqualityConstraintPtr(other.finalEqualityConstraintPtr->Clone()),
      /* Inequality constraints */
      inequalityConstraintPtr(other.inequalityConstraintPtr->Clone()),
      stateInequalityConstraintPtr(other.stateInequalityConstraintPtr->Clone()),
      preJumpInequalityConstraintPtr(other.preJumpInequalityConstraintPtr->Clone()),
      finalInequalityConstraintPtr(other.finalInequalityConstraintPtr->Clone()),
      /* Lagrangians */
      equalityLagrangianPtr(other.equalityLagrangianPtr->Clone()),
      stateEqualityLagrangianPtr(other.stateEqualityLagrangianPtr->Clone()),
      inequalityLagrangianPtr(other.inequalityLagrangianPtr->Clone()),
      stateInequalityLagrangianPtr(other.stateInequalityLagrangianPtr->Clone()),
      preJumpEqualityLagrangianPtr(other.preJumpEqualityLagrangianPtr->Clone()),
      preJumpInequalityLagrangianPtr(other.preJumpInequalityLagrangianPtr->Clone()),
      finalEqualityLagrangianPtr(other.finalEqualityLagrangianPtr->Clone()),
      finalInequalityLagrangianPtr(other.finalInequalityLagrangianPtr->Clone()),
      /* Misc. */
      preComputationPtr(other.preComputationPtr->Clone()),
      targetTrajectoriesPtr(other.targetTrajectoriesPtr) {
  if (other.dynamicsPtr != nullptr) {
    dynamicsPtr.reset(other.dynamicsPtr->Clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem& OptimalControlProblem::operator=(const OptimalControlProblem& rhs) {
  OptimalControlProblem tmp(rhs);
  swap(tmp);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OptimalControlProblem::swap(OptimalControlProblem& other) noexcept {
  /* Cost */
  costPtr.swap(other.costPtr);
  stateCostPtr.swap(other.stateCostPtr);
  preJumpCostPtr.swap(other.preJumpCostPtr);
  finalCostPtr.swap(other.finalCostPtr);

  /* Soft constraints */
  softConstraintPtr.swap(other.softConstraintPtr);
  stateSoftConstraintPtr.swap(other.stateSoftConstraintPtr);
  preJumpSoftConstraintPtr.swap(other.preJumpSoftConstraintPtr);
  finalSoftConstraintPtr.swap(other.finalSoftConstraintPtr);

  /* Equality constraints */
  equalityConstraintPtr.swap(other.equalityConstraintPtr);
  stateEqualityConstraintPtr.swap(other.stateEqualityConstraintPtr);
  preJumpEqualityConstraintPtr.swap(other.preJumpEqualityConstraintPtr);
  finalEqualityConstraintPtr.swap(other.finalEqualityConstraintPtr);

  /* Inequality constraints */
  inequalityConstraintPtr.swap(other.inequalityConstraintPtr);
  stateInequalityConstraintPtr.swap(other.stateInequalityConstraintPtr);
  preJumpInequalityConstraintPtr.swap(other.preJumpInequalityConstraintPtr);
  finalInequalityConstraintPtr.swap(other.finalInequalityConstraintPtr);

  /* Lagrangians */
  equalityLagrangianPtr.swap(other.equalityLagrangianPtr);
  stateEqualityLagrangianPtr.swap(other.stateEqualityLagrangianPtr);
  inequalityLagrangianPtr.swap(other.inequalityLagrangianPtr);
  stateInequalityLagrangianPtr.swap(other.stateInequalityLagrangianPtr);
  preJumpEqualityLagrangianPtr.swap(other.preJumpEqualityLagrangianPtr);
  preJumpInequalityLagrangianPtr.swap(other.preJumpInequalityLagrangianPtr);
  finalEqualityLagrangianPtr.swap(other.finalEqualityLagrangianPtr);
  finalInequalityLagrangianPtr.swap(other.finalInequalityLagrangianPtr);

  /* Dynamics */
  dynamicsPtr.swap(other.dynamicsPtr);

  /* Misc. */
  preComputationPtr.swap(other.preComputationPtr);
  std::swap(targetTrajectoriesPtr, other.targetTrajectoriesPtr);
}

}  // namespace bolza
