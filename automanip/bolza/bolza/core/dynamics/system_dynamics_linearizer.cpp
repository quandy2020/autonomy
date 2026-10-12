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

#include <bolza/core/dynamics/system_dynamics_linearizer.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer::SystemDynamicsLinearizer(std::unique_ptr<ControlledSystemBase> nonlinearSystemPtr,
                                                   bool doubleSidedDerivative /*= true*/, bool isSecondOrderSystem /*= false*/,
                                                   scalar_t eps /*= Eigen::NumTraits<scalar_t>::epsilon()*/)
    : SystemDynamicsBase(nonlinearSystemPtr->getPreComputation()),
      controlledSystemPtr_(std::move(nonlinearSystemPtr)),
      doubleSidedDerivative_(doubleSidedDerivative),
      isSecondOrderSystem_(isSecondOrderSystem),
      eps_(eps) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer::SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other)
    : SystemDynamicsBase(other),
      controlledSystemPtr_(other.controlledSystemPtr_->clone()),
      doubleSidedDerivative_(other.doubleSidedDerivative_),
      isSecondOrderSystem_(other.isSecondOrderSystem_),
      eps_(other.eps_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer* SystemDynamicsLinearizer::Clone() const {
  return new SystemDynamicsLinearizer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsLinearizer::ComputeFlowMap(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const PreComputation& preComp) {
  return controlledSystemPtr_->computeFlowMap(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsLinearizer::LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                const PreComputation& preComp) {
  VectorFunctionLinearApproximation linearDynamics;
  linearDynamics.f = controlledSystemPtr_->computeFlowMap(t, x, u, preComp);
  linearDynamics.dfdx = FiniteDifferenceDerivativeState(*controlledSystemPtr_, t, x, u, eps_, doubleSidedDerivative_, isSecondOrderSystem_);
  linearDynamics.dfdu = FiniteDifferenceDerivativeInput(*controlledSystemPtr_, t, x, u, eps_, doubleSidedDerivative_, isSecondOrderSystem_);
  return linearDynamics;
}

}  // namespace bolza
