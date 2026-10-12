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

#include <bolza/core/dynamics/controlled_system_base.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const PreComputation& preComputation) : preCompPtr_(preComputation.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const ControlledSystemBase& other) : OdeBase(other) {
  assert(other.preCompPtr_ != nullptr);
  preCompPtr_.reset(other.preCompPtr_->clone());
  SetController(other.controllerPtr());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::ComputeFlowMap(scalar_t t, const vector_t& x) {
  assert(controllerPtr_ != nullptr);
  const vector_t u = controllerPtr_->computeInput(t, x);
  return ComputeFlowMap(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->request(Request::Dynamics, t, x, u);
  return ComputeFlowMap(t, x, u, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::ComputeJumpMap(scalar_t t, const vector_t& x) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->requestPreJump(Request::Dynamics, t, x);
  return ComputeJumpMap(t, x, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) {
  // default implementation
  return OdeBase::ComputeJumpMap(t, x);
}

}  // namespace bolza
