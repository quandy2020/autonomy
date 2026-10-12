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

#include "bolza/pinocchio/centroidal/pinocchio_centroidal_dynamics_ad.hpp"

#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamicsAD::PinocchioCentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info,
                                                             const std::string& modelName, const std::string& modelFolder,
                                                             bool recompileLibraries, bool verbose) {
  auto systemFlowMapFunc = [&](const ad_vector_t& x, ad_vector_t& y) {
    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

    // mapping
    CentroidalModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
    mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

    ad_vector_t state = x.head(info.stateDim);
    ad_vector_t input = x.tail(info.inputDim);
    y = GetValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
  };

  systemFlowMapCppAdInterfacePtr_.reset(
      new CppAdInterface(systemFlowMapFunc, info.stateDim + info.inputDim, modelName + "_systemFlowMap", modelFolder));

  if (recompileLibraries) {
    systemFlowMapCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamicsAD::PinocchioCentroidalDynamicsAD(const PinocchioCentroidalDynamicsAD& rhs)
    : systemFlowMapCppAdInterfacePtr_(new CppAdInterface(*rhs.systemFlowMapCppAdInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioCentroidalDynamicsAD::GetValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                         const CentroidalModelPinocchioMappingCppAd& mappingCppAd, const ad_vector_t& state,
                                                         const ad_vector_t& input) {
  const auto& info = mappingCppAd.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());

  const ad_vector_t qPinocchio = mappingCppAd.getPinocchioJointPosition(state);
  UpdateCentroidalDynamics(pinocchioInterfaceCppAd, info, qPinocchio);

  ad_vector_t stateDerivative(info.stateDim);

  // compute center of mass acceleration and derivative of the normalized angular momentum
  centroidal_model::GetNormalizedMomentum(stateDerivative, info) =
      GetNormalizedCentroidalMomentumRate(pinocchioInterfaceCppAd, info, input);

  // derivatives of the floating base variables + joint velocities
  centroidal_model::getGeneralizedCoordinates(stateDerivative, info) = mappingCppAd.getPinocchioJointVelocity(state, input);

  return stateDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalDynamicsAD::GetValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
  return systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioCentroidalDynamicsAD::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input) const {
  const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
  VectorFunctionLinearApproximation approx;
  approx.f = systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
  const matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(stateInput);
  approx.dfdx = dynamicsJacobian.leftCols(state.rows());
  approx.dfdu = dynamicsJacobian.rightCols(input.rows());
  return approx;
}

}  // namespace bolza
