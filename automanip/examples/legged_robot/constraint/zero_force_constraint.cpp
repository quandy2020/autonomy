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

#include "examples/legged_robot/constraint/zero_force_constraint.hpp"

#include <bolza/pinocchio/centroidal/access_helper_functions.hpp>

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,
                                         CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroForceConstraint::IsActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroForceConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  return centroidal_model::GetContactForces(input, contactPointIndex_, info_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroForceConstraint::GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = GetValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(3, state.size());
  approx.dfdu = matrix_t::Zero(3, input.size());
  approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = vector_t::Ones(3);
  return approx;
}

}  // namespace legged_robot
}  // namespace bolza
