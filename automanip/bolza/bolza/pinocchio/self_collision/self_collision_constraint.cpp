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

#include <bolza/tools/common/rotation_transforms.hpp>
#include "bolza/pinocchio/self_collision/self_collision_constraint.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraint::SelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                                 PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      selfCollision_(std::move(pinocchioGeometryInterface), minimumDistance),
      mappingPtr_(mapping.Clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraint::SelfCollisionConstraint(const SelfCollisionConstraint& rhs)
    : StateConstraint(rhs), selfCollision_(rhs.selfCollision_), mappingPtr_(rhs.mappingPtr_->Clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t SelfCollisionConstraint::GetNumConstraints(scalar_t time) const {
  return selfCollision_.GetNumCollisionPairs();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionConstraint::GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  const auto& pinocchioInterface = GetPinocchioInterface(preComputation);
  return selfCollision_.GetValue(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SelfCollisionConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComputation) const {
  const auto& pinocchioInterface = GetPinocchioInterface(preComputation);
  mappingPtr_->SetPinocchioInterface(pinocchioInterface);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = selfCollision_.GetLinearApproximation(pinocchioInterface);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->GetOcs2Jacobian(state, dfdq, dfdv);
  return constraint;
}

}  // namespace bolza
