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

#include "bolza/ipm/ipm_initialization.hpp"

namespace bolza {
namespace ipm {

std::pair<vector_t, vector_t> InitializeIntermediateSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time,
                                                                  const vector_t& state, const vector_t& input,
                                                                  scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  vector_t slackStateIneq, slackStateInputIneq;

  if (!ocpDefinition.stateInequalityConstraintPtr->empty() || !ocpDefinition.inequalityConstraintPtr->empty()) {
    constexpr auto request = Request::Constraint;
    ocpDefinition.preComputationPtr->Request(request, time, state, input);
  }

  if (!ocpDefinition.stateInequalityConstraintPtr->empty()) {
    const auto ineqConstraint =
        ToVector(ocpDefinition.stateInequalityConstraintPtr->GetValue(time, state, *ocpDefinition.preComputationPtr));
    slackStateIneq = InitializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
  }

  if (!ocpDefinition.inequalityConstraintPtr->empty()) {
    const auto ineqConstraint =
        ToVector(ocpDefinition.inequalityConstraintPtr->GetValue(time, state, input, *ocpDefinition.preComputationPtr));
    slackStateInputIneq = InitializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
  }

  return std::make_pair(std::move(slackStateIneq), std::move(slackStateInputIneq));
}

vector_t InitializeTerminalSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                         scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ocpDefinition.finalInequalityConstraintPtr->empty()) {
    return vector_t();
  }

  constexpr auto request = Request::Constraint;
  ocpDefinition.preComputationPtr->RequestFinal(request, time, state);
  const auto ineqConstraint = ToVector(ocpDefinition.finalInequalityConstraintPtr->GetValue(time, state, *ocpDefinition.preComputationPtr));
  return InitializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
}

vector_t InitializeEventSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                      scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ocpDefinition.preJumpInequalityConstraintPtr->empty()) {
    return vector_t();
  }

  constexpr auto request = Request::Constraint;
  ocpDefinition.preComputationPtr->RequestPreJump(request, time, state);
  const auto ineqConstraint =
      ToVector(ocpDefinition.preJumpInequalityConstraintPtr->GetValue(time, state, *ocpDefinition.preComputationPtr));
  return InitializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
}

}  // namespace ipm
}  // namespace bolza