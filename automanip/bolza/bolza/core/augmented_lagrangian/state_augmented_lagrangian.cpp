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

#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr,
                                                   std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr,
                                                   std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(const StateAugmentedLagrangian& other)
    : StateAugmentedLagrangianInterface(other), constraintPtr_(other.constraintPtr_->Clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian* StateAugmentedLagrangian::Clone() const {
  return new StateAugmentedLagrangian(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateAugmentedLagrangian::IsActive(scalar_t time) const {
  return constraintPtr_->IsActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateAugmentedLagrangian::GetNumConstraints(scalar_t time) const {
  return constraintPtr_->GetNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LagrangianMetrics StateAugmentedLagrangian::GetValue(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                                                     const PreComputation& preComp) const {
  const auto h = constraintPtr_->GetValue(time, state, preComp);
  const auto p = multiplier.penalty * penalty_.GetValue(time, h, &multiplier.lagrangian);
  return {p, h};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateAugmentedLagrangian::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const Multiplier& multiplier,
                                                                                         const PreComputation& preComp) const {
  switch (constraintPtr_->GetOrder()) {
    case ConstraintOrder::Linear:
      return multiplier.penalty *
             penalty_.GetQuadraticApproximation(time, constraintPtr_->GetLinearApproximation(time, state, preComp), &multiplier.lagrangian);
    case ConstraintOrder::Quadratic:
      return multiplier.penalty * penalty_.GetQuadraticApproximation(time, constraintPtr_->GetQuadraticApproximation(time, state, preComp),
                                                                     &multiplier.lagrangian);
    default:
      throw std::runtime_error("[StateAugmentedLagrangian] Unknown constraint Order");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Multiplier, scalar_t> StateAugmentedLagrangian::UpdateLagrangian(scalar_t time, const vector_t& /*state*/,
                                                                           const vector_t& constraint, const Multiplier& multiplier) const {
  const Multiplier updatedMultiplier{multiplier.penalty, penalty_.UpdateMultipliers(time, constraint, multiplier.lagrangian)};
  const auto penalty = updatedMultiplier.penalty * penalty_.GetValue(time, constraint, &updatedMultiplier.lagrangian);
  return {updatedMultiplier, penalty};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Multiplier StateAugmentedLagrangian::InitializeLagrangian(scalar_t time) const {
  return {1.0, penalty_.InitializeMultipliers(GetNumConstraints(time))};
}

}  // namespace bolza
