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

#include "examples/legged_robot/constraint/end_effector_linear_constraint.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t numConstraints, Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.Clone()),
      numConstraints_(numConstraints),
      config_(std::move(config)) {
  if (endEffectorKinematicsPtr_->GetIds().size() != 1) {
    throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs)
    : StateInputConstraint(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->Clone()),
      numConstraints_(rhs.numConstraints_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorLinearConstraint::Configure(Config&& config) {
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
  config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLinearConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  vector_t f = config_.b;
  if (config_.Ax.size() > 0) {
    f.noalias() += config_.Ax * endEffectorKinematicsPtr_->GetPosition(state).front();
  }
  if (config_.Av.size() > 0) {
    f.noalias() += config_.Av * endEffectorKinematicsPtr_->GetVelocity(state, input).front();
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorLinearConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(GetNumConstraints(time), state.size(), input.size());

  linearApproximation.f = config_.b;

  if (config_.Ax.size() > 0) {
    const auto positionApprox = endEffectorKinematicsPtr_->GetPositionLinearApproximation(state).front();
    linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
    linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
  }

  if (config_.Av.size() > 0) {
    const auto velocityApprox = endEffectorKinematicsPtr_->GetVelocityLinearApproximation(state, input).front();
    linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
    linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
    linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace bolza
