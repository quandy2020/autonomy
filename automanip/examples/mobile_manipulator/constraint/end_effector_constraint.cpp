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

#include "examples/mobile_manipulator/mobile_manipulator_pre_computation.hpp"
#include "examples/mobile_manipulator/constraint/end_effector_constraint.hpp"

#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const ReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.Clone()),
      referenceManagerPtr_(&referenceManager) {
  if (endEffectorKinematics.GetIds().size() != 1) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::GetNumConstraints(scalar_t time) const {
  return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->SetPinocchioInterface(preCompMM.GetPinocchioInterface());
  }

  const auto desiredPositionOrientation = InterpolateEndEffectorPose(time);

  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsPtr_->GetPosition(state).front() - desiredPositionOrientation.first;
  constraint.tail<3>() = endEffectorKinematicsPtr_->GetOrientationError(state, {desiredPositionOrientation.second}).front();
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->SetPinocchioInterface(preCompMM.GetPinocchioInterface());
  }

  const auto desiredPositionOrientation = InterpolateEndEffectorPose(time);

  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->GetPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;

  const auto eeOrientationError =
      endEffectorKinematicsPtr_->GetOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  approximation.f.tail<3>() = eeOrientationError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::InterpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManagerPtr_->GetTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::TimeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index];
    const auto& rhs = stateTrajectory[index + 1];
    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // stateTrajectory.size() == 1
    position = stateTrajectory.front().head<3>();
    orientation = quaternion_t(stateTrajectory.front().tail<4>());
  }

  return {position, orientation};
}

}  // namespace mobile_manipulator
}  // namespace bolza
