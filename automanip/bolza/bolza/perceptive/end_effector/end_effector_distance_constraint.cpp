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

#include "bolza/perceptive/end_effector/end_effector_distance_constraint.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight,
                                                             std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      weight_(weight),
      kinematicsPtr_(std::move(kinematicsPtr)),
      clearances_(kinematicsPtr_->GetIds().size(), 0.0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight,
                                                             std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr,
                                                             scalar_array_t clearances)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      weight_(weight),
      kinematicsPtr_(std::move(kinematicsPtr)),
      clearances_(std::move(clearances)) {
  if (clearances_.size() != kinematicsPtr_->GetIds().size()) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] clearances.size() != kinematicsPtr->GetIds().size()");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint& other)
    : StateConstraint(other),
      stateDim_(other.stateDim_),
      weight_(other.weight_),
      kinematicsPtr_(other.kinematicsPtr_->Clone()),
      clearances_(other.clearances_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(const DistanceTransformInterface& distanceTransform) {
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(scalar_t clearance, const DistanceTransformInterface& distanceTransform) {
  clearances_.assign(kinematicsPtr_->GetIds().size(), clearance);
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(const scalar_array_t& clearances, const DistanceTransformInterface& distanceTransform) {
  assert(clearances.size() == kinematicsPtr_->GetIds().size());
  clearances_ = clearances;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorDistanceConstraint::GetValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const {
  if (distanceTransformPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
  }

  const auto numEEs = kinematicsPtr_->GetIds().size();
  const auto eePositions = kinematicsPtr_->GetPosition(state);

  vector_t g(numEEs);
  for (size_t i = 0; i < numEEs; i++) {
    g(i) = weight_ * (distanceTransformPtr_->GetValue(eePositions[i]) - clearances_[i]);
  }  // end of i loop

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorDistanceConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const PreComputation& preComp) const {
  if (distanceTransformPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
  }

  const auto numEEs = kinematicsPtr_->GetIds().size();
  const auto eePosLinApprox = kinematicsPtr_->GetPositionLinearApproximation(state);

  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, 0);
  for (size_t i = 0; i < numEEs; i++) {
    const auto distanceValueGradient = distanceTransformPtr_->GetLinearApproximation(eePosLinApprox[i].f);
    approx.f(i) = weight_ * (distanceValueGradient.first - clearances_[i]);
    approx.dfdx.row(i).noalias() = weight_ * (distanceValueGradient.second.transpose() * eePosLinApprox[i].dfdx);
  }  // end of i loop

  return approx;
}

}  // namespace bolza
