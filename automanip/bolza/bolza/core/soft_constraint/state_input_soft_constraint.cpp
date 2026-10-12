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

#include <bolza/core/soft_constraint/state_input_soft_constraint.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftConstraint::StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                   std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftConstraint::StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                   std::unique_ptr<PenaltyBase> penaltyFunction)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyFunction)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftConstraint::StateInputSoftConstraint(const StateInputSoftConstraint& other)
    : StateInputCost(other), constraintPtr_(other.constraintPtr_->Clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftConstraint* StateInputSoftConstraint::Clone() const {
  return new StateInputSoftConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateInputSoftConstraint::IsActive(scalar_t time) const {
  return constraintPtr_->IsActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputSoftConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories&,
                                            const PreComputation& preComp) const {
  return penalty_.GetValue(time, constraintPtr_->GetValue(time, state, input, preComp));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputSoftConstraint::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const vector_t& input, const TargetTrajectories&,
                                                                                         const PreComputation& preComp) const {
  switch (constraintPtr_->GetOrder()) {
    case ConstraintOrder::Linear:
      return penalty_.GetQuadraticApproximation(time, constraintPtr_->GetLinearApproximation(time, state, input, preComp));
    case ConstraintOrder::Quadratic:
      return penalty_.GetQuadraticApproximation(time, constraintPtr_->GetQuadraticApproximation(time, state, input, preComp));
    default:
      throw std::runtime_error("[StateInputSoftConstraint] Unknown constraint Order");
  }
}

}  // namespace bolza
