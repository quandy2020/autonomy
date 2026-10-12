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

#include "examples/legged_robot/constraint/normal_velocity_constraint_cpp_ad.hpp"
#include "examples/legged_robot/legged_robot_pre_computation.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->Clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraintCppAd::IsActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NormalVelocityConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->GetValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->GetLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace bolza
