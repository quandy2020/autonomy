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

#include <bolza/core/cost/state_input_gauss_newton_cost_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputCostGaussNewtonAd::Initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                             const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto costVectorAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->costVectorFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new bolza::CppAdInterface(costVectorAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

  if (recompileLibraries) {
    adInterfacePtr_->createModels(bolza::CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(bolza::CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostGaussNewtonAd::StateInputCostGaussNewtonAd(const StateInputCostGaussNewtonAd& rhs)
    : StateInputCost(rhs), adInterfacePtr_(new bolza::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostGaussNewtonAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
  vector_t timeStateInput(1 + state.rows() + input.rows());
  timeStateInput << time, state, input;
  const auto parameters = GetParameters(time, targetTrajectories, preComputation);
  const auto costVector = adInterfacePtr_->getFunctionValue(timeStateInput, parameters);
  return 0.5 * costVector.squaredNorm();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostGaussNewtonAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const TargetTrajectories& targetTrajectories,
                                                                                            const PreComputation& preComputation) const {
  const auto stateDim = state.rows();
  const auto inputDim = input.rows();
  vector_t timeStateInput(1 + stateDim + inputDim);
  timeStateInput << time, state, input;
  const auto parameters = GetParameters(time, targetTrajectories, preComputation);
  const auto gnApproximation = adInterfacePtr_->getGaussNewtonApproximation(timeStateInput, parameters);

  ScalarFunctionQuadraticApproximation L;
  L.f = gnApproximation.f;
  L.dfdx.noalias() = gnApproximation.dfdx.middleRows(1, stateDim);
  L.dfdu.noalias() = gnApproximation.dfdx.bottomRows(inputDim);
  L.dfdxx = gnApproximation.dfdxx.block(1, 1, stateDim, stateDim);
  L.dfdux.noalias() = gnApproximation.dfdxx.block(1 + stateDim, 1, inputDim, stateDim);
  L.dfduu.noalias() = gnApproximation.dfdxx.block(1 + stateDim, 1 + stateDim, inputDim, inputDim);
  return L;
}

}  // namespace bolza
