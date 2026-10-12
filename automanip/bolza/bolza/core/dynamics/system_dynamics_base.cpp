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

#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBase::SystemDynamicsBase(const PreComputation& preComputation) : ControlledSystemBase(preComputation) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBase::SystemDynamicsBase(const SystemDynamicsBase& other) : ControlledSystemBase(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->request(Request::Dynamics + Request::Approximation, t, x, u);
  return LinearApproximation(t, x, u, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::JumpMapLinearApproximation(scalar_t t, const vector_t& x) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->requestPreJump(Request::Dynamics + Request::Approximation, t, x);
  return JumpMapLinearApproximation(t, x, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::JumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                 const PreComputation& preComp) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx.setIdentity(x.rows(), x.rows());
  approximation.dfdu.setZero(x.rows(), 0);
  assert(preCompPtr_ != nullptr);
  approximation.f = ControlledSystemBase::ComputeJumpMap(t, x, preComp);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx.setZero(1, x.rows());
  approximation.dfdu.setZero(1, u.rows());
  approximation.f = this->computeGuardSurfaces(t, x);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(x.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(x.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBase::DynamicsCovariance(scalar_t t, const vector_t& x, const vector_t& u) {
  return matrix_t::Zero(0, 0);
}

}  // namespace bolza
