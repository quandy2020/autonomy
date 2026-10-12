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

#include "examples/legged_robot/constraint/zero_velocity_constraint_cpp_ad.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t contactPointIndex, EndEffectorLinearConstraint::Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 3, std::move(config))),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->Clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroVelocityConstraintCppAd::IsActive(scalar_t time) const {
  return referenceManagerPtr_->GetContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroVelocityConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  return eeLinearConstraintPtr_->GetValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroVelocityConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  return eeLinearConstraintPtr_->GetLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace bolza
