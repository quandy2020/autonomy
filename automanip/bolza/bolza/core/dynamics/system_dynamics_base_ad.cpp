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

#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD() : SystemDynamicsBase() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs)
    : SystemDynamicsBase(rhs),
      flowMapADInterfacePtr_(new CppAdInterface(*rhs.flowMapADInterfacePtr_)),
      jumpMapADInterfacePtr_(new CppAdInterface(*rhs.jumpMapADInterfacePtr_)),
      guardSurfacesADInterfacePtr_(new CppAdInterface(*rhs.guardSurfacesADInterfacePtr_)),
      tapedTimeStateInput_(rhs.tapedTimeStateInput_.size()),
      tapedTimeState_(rhs.tapedTimeState_.size()),
      flowJacobian_(rhs.flowJacobian_.rows(), rhs.flowJacobian_.cols()),
      jumpJacobian_(rhs.jumpJacobian_.rows(), rhs.jumpJacobian_.cols()),
      guardJacobian_(rhs.guardJacobian_.rows(), rhs.guardJacobian_.cols()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::Initialize(size_t stateDim, size_t inputDim, const std::string& modelName, const std::string& modelFolder,
                                      bool recompileLibraries, bool verbose) {
  tapedTimeStateInput_.resize(1 + stateDim + inputDim);
  tapedTimeState_.resize(1 + stateDim);

  auto flowMap = [this, stateDim, inputDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->systemFlowMap(time, state, input, p);
  };
  flowMapADInterfacePtr_.reset(
      new CppAdInterface(flowMap, 1 + stateDim + inputDim, GetNumFlowMapParameters(), modelName + "_flow_map", modelFolder));

  auto jumpMap = [this, stateDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->systemJumpMap(time, state, p);
  };
  jumpMapADInterfacePtr_.reset(new CppAdInterface(jumpMap, 1 + stateDim, GetNumJumpMapParameters(), modelName + "_jump_map", modelFolder));

  auto guardSurfaces = [this, stateDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->systemGuardSurfaces(time, state, p);
  };
  guardSurfacesADInterfacePtr_.reset(
      new CppAdInterface(guardSurfaces, 1 + stateDim, GetNumGuardSurfacesParameters(), modelName + "_guard_surfaces", modelFolder));

  if (recompileLibraries) {
    flowMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    jumpMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    guardSurfacesADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    flowMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    jumpMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    guardSurfacesADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComputation) {
  tapedTimeStateInput_ << t, x, u;
  const vector_t parameters = GetFlowMapParameters(t, preComputation);
  return flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, parameters);
}

/*******************q**********************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComputation) {
  tapedTimeState_ << t, x;
  const vector_t parameters = GetJumpMapParameters(t, preComputation);
  return jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::ComputeGuardSurfaces(scalar_t t, const vector_t& x) {
  tapedTimeState_ << t, x;
  const vector_t parameters = GetGuardSurfacesParameters(t);
  return guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                            const PreComputation& preComputation) {
  tapedTimeStateInput_ << t, x, u;
  const vector_t parameters = GetFlowMapParameters(t, preComputation);
  flowJacobian_ = flowMapADInterfacePtr_->getJacobian(tapedTimeStateInput_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = flowJacobian_.middleCols(1, x.rows());
  approximation.dfdu = flowJacobian_.rightCols(u.rows());
  approximation.f = flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::JumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                   const PreComputation& preComputation) {
  tapedTimeState_ << t, x;
  const vector_t parameters = GetJumpMapParameters(t, preComputation);
  jumpJacobian_ = jumpMapADInterfacePtr_->getJacobian(tapedTimeState_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = jumpJacobian_.rightCols(x.rows());
  approximation.dfdu.setZero(jumpJacobian_.rows(), 0);
  approximation.f = jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeState_ << t, x;
  const vector_t parameters = GetGuardSurfacesParameters(t);
  guardJacobian_ = guardSurfacesADInterfacePtr_->getJacobian(tapedTimeState_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = guardJacobian_.rightCols(x.rows());
  approximation.dfdu = matrix_t::Zero(guardJacobian_.rows(), u.rows());  // not provided
  approximation.f = guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return flowJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return jumpJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return guardJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SystemDynamicsBaseAD::SystemJumpMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SystemDynamicsBaseAD::SystemGuardSurfaces(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  return -ad_vector_t::Ones(1);
}

}  // namespace bolza
