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

#include <bolza/core/soft_constraint/state_soft_constraint.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr,
                                         std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr, std::unique_ptr<PenaltyBase> penaltyFunction)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyFunction)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(const StateSoftConstraint& other)
    : StateCost(other), constraintPtr_(other.constraintPtr_->clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint* StateSoftConstraint::Clone() const {
  return new StateSoftConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateSoftConstraint::IsActive(scalar_t time) const {
  return constraintPtr_->isActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateSoftConstraint::GetValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                                       const PreComputation& preComp) const {
  return penalty_.getValue(time, constraintPtr_->getValue(time, state, preComp));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateSoftConstraint::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                    const TargetTrajectories&,
                                                                                    const PreComputation& preComp) const {
  switch (constraintPtr_->getOrder()) {
    case ConstraintOrder::Linear:
      return penalty_.getQuadraticApproximation(time, constraintPtr_->getLinearApproximation(time, state, preComp));
    case ConstraintOrder::Quadratic:
      return penalty_.getQuadraticApproximation(time, constraintPtr_->getQuadraticApproximation(time, state, preComp));
    default:
      throw std::runtime_error("[StateSoftConstraint] Unknown constraint Order");
  }
}

}  // namespace bolza
