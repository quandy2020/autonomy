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

#pragma once

#include <memory>

#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics.hpp>
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

namespace bolza {
namespace mobile_manipulator {

class EndEffectorConstraint final : public StateConstraint {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager);
  ~EndEffectorConstraint() override = default;
  EndEffectorConstraint* Clone() const override { return new EndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_); }

  size_t GetNumConstraints(scalar_t time) const override;
  vector_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

 private:
  EndEffectorConstraint(const EndEffectorConstraint& other) = default;
  std::pair<vector_t, quaternion_t> InterpolateEndEffectorPose(scalar_t time) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const ReferenceManager* referenceManagerPtr_;
};

}  // namespace mobile_manipulator
}  // namespace bolza
