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

#include <bolza/core/augmented_lagrangian/state_input_augmented_lagrangian.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangian::StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                             std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangian::StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                             std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangian::StateInputAugmentedLagrangian(const StateInputAugmentedLagrangian& other)
    : StateInputAugmentedLagrangianInterface(other), constraintPtr_(other.constraintPtr_->clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangian* StateInputAugmentedLagrangian::Clone() const {
  return new StateInputAugmentedLagrangian(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateInputAugmentedLagrangian::IsActive(scalar_t time) const {
  return constraintPtr_->isActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputAugmentedLagrangian::GetNumConstraints(scalar_t time) const {
  return constraintPtr_->getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LagrangianMetrics StateInputAugmentedLagrangian::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                          const Multiplier& multiplier, const PreComputation& preComp) const {
  const auto h = constraintPtr_->getValue(time, state, input, preComp);
  const auto p = multiplier.penalty * penalty_.getValue(time, h, &multiplier.lagrangian);
  return {p, h};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputAugmentedLagrangian::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                              const vector_t& input,
                                                                                              const Multiplier& multiplier,
                                                                                              const PreComputation& preComp) const {
  switch (constraintPtr_->getOrder()) {
    case ConstraintOrder::Linear:
      return multiplier.penalty * penalty_.getQuadraticApproximation(
                                      time, constraintPtr_->getLinearApproximation(time, state, input, preComp), &multiplier.lagrangian);
    case ConstraintOrder::Quadratic:
      return multiplier.penalty * penalty_.getQuadraticApproximation(
                                      time, constraintPtr_->getQuadraticApproximation(time, state, input, preComp), &multiplier.lagrangian);
    default:
      throw std::runtime_error("[StateInputAugmentedLagrangian] Unknown constraint Order");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Multiplier, scalar_t> StateInputAugmentedLagrangian::UpdateLagrangian(scalar_t time, const vector_t& /*state*/,
                                                                                const vector_t& /*input*/, const vector_t& constraint,
                                                                                const Multiplier& multiplier) const {
  const Multiplier updatedMultiplier{multiplier.penalty, penalty_.updateMultipliers(time, constraint, multiplier.lagrangian)};
  const auto penalty = updatedMultiplier.penalty * penalty_.getValue(time, constraint, &updatedMultiplier.lagrangian);
  return {updatedMultiplier, penalty};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Multiplier StateInputAugmentedLagrangian::InitializeLagrangian(scalar_t time) const {
  return {1.0, penalty_.initializeMultipliers(GetNumConstraints(time))};
}

}  // namespace bolza
