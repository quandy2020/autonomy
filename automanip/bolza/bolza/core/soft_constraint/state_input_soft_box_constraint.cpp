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

#include <bolza/core/soft_constraint/state_input_soft_box_constraint.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftBoxConstraint::BoxConstraint::BoxConstraint(const BoxConstraint& other)
    : index(other.index), lowerBound(other.lowerBound), upperBound(other.upperBound), penaltyPtr(other.penaltyPtr->Clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftBoxConstraint::BoxConstraint& StateInputSoftBoxConstraint::BoxConstraint::operator=(const BoxConstraint& other) {
  this->index = other.index;
  this->lowerBound = other.lowerBound;
  this->upperBound = other.upperBound;
  this->penaltyPtr.reset(other.penaltyPtr->Clone());
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftBoxConstraint::StateInputSoftBoxConstraint(std::vector<BoxConstraint> stateBoxConstraints,
                                                         std::vector<BoxConstraint> inputBoxConstraints)
    : stateBoxConstraints_(std::move(stateBoxConstraints)), inputBoxConstraints_(std::move(inputBoxConstraints)), offset_(0.0) {
  SortByIndex(stateBoxConstraints_);
  SortByIndex(inputBoxConstraints_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftBoxConstraint* StateInputSoftBoxConstraint::Clone() const {
  return new StateInputSoftBoxConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateInputSoftBoxConstraint::IsActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftBoxConstraint::SortByIndex(std::vector<BoxConstraint>& boxConstraints) const {
  std::sort(boxConstraints.begin(), boxConstraints.end(),
            [](const BoxConstraint& lhs, const BoxConstraint& rhs) { return lhs.index < rhs.index; });
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftBoxConstraint::InitializeOffset(scalar_t time, const vector_t& state, const vector_t& input) {
  offset_ = -GetValue(time, state, stateBoxConstraints_) - GetValue(time, input, inputBoxConstraints_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputSoftBoxConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories&,
                                               const PreComputation& preComp) const {
  return GetValue(time, state, stateBoxConstraints_) + GetValue(time, input, inputBoxConstraints_) + offset_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputSoftBoxConstraint::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const TargetTrajectories&,
                                                                                            const PreComputation& preComp) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
  FillQuadraticApproximation(time, state, stateBoxConstraints_, cost.f, cost.dfdx, cost.dfdxx);
  FillQuadraticApproximation(time, input, inputBoxConstraints_, cost.f, cost.dfdu, cost.dfduu);
  cost.f += offset_;
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputSoftBoxConstraint::GetValue(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints) const {
  scalar_t f = scalar_t(0.0);
  for (const auto& boxConstraint : boxConstraints) {
    const auto i = boxConstraint.index;
    const auto& p = *boxConstraint.penaltyPtr;
    const scalar_t upperBoundDistance = boxConstraint.upperBound - h(i);
    const scalar_t lowerBoundDistance = h(i) - boxConstraint.lowerBound;

    f += p.GetValue(t, lowerBoundDistance) + p.GetValue(t, upperBoundDistance);
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftBoxConstraint::FillQuadraticApproximation(scalar_t t, const vector_t& h,
                                                             const std::vector<BoxConstraint>& boxConstraints, scalar_t& value,
                                                             vector_t& firstDerivative, matrix_t& secondDerivative) const {
  for (const auto& boxConstraint : boxConstraints) {
    const auto i = boxConstraint.index;
    const auto& p = *boxConstraint.penaltyPtr;
    const scalar_t upperBoundDistance = boxConstraint.upperBound - h(i);
    const scalar_t lowerBoundDistance = h(i) - boxConstraint.lowerBound;

    value += p.GetValue(t, lowerBoundDistance) + p.GetValue(t, upperBoundDistance);
    firstDerivative(i) += p.GetDerivative(t, lowerBoundDistance) - p.GetDerivative(t, upperBoundDistance);
    secondDerivative(i, i) += p.GetSecondDerivative(t, lowerBoundDistance) + p.GetSecondDerivative(t, upperBoundDistance);
  }
}

}  // namespace bolza
