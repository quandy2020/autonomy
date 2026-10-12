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

#include <bolza/core/constraint/state_input_constraint.hpp>

#include "examples/legged_robot/constraint/end_effector_linear_constraint.hpp"
#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

namespace bolza {
namespace legged_robot {

/**
 * Specializes the CppAd version of zero velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorLinearConstraint object with number of constraints of 3.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class ZeroVelocityConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] config: The constraint coefficients
   */
  ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                              const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                              EndEffectorLinearConstraint::Config config = EndEffectorLinearConstraint::Config());

  ~ZeroVelocityConstraintCppAd() override = default;
  ZeroVelocityConstraintCppAd* Clone() const override { return new ZeroVelocityConstraintCppAd(*this); }

  bool IsActive(scalar_t time) const override;
  size_t GetNumConstraints(scalar_t time) const override { return 3; }
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
  const size_t contactPointIndex_;
};

}  // namespace legged_robot
}  // namespace bolza
