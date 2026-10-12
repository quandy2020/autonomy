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

#include <bolza/core/cost/state_input_cost_cpp_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputCostCppAd::Initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                     const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto costAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = ad_vector_t(1);
    y(0) = this->costFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new bolza::CppAdInterface(costAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

  if (recompileLibraries) {
    adInterfacePtr_->createModels(bolza::CppAdInterface::ApproximationOrder::Second, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(bolza::CppAdInterface::ApproximationOrder::Second, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCppAd::StateInputCostCppAd(const StateInputCostCppAd& rhs)
    : StateInputCost(rhs), adInterfacePtr_(new bolza::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostCppAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  tapedTimeStateInput << time, state, input;
  return adInterfacePtr_->getFunctionValue(tapedTimeStateInput, GetParameters(time, targetTrajectories, preComputation))(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostCppAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input,
                                                                                    const TargetTrajectories& targetTrajectories,
                                                                                    const PreComputation& preComputation) const {
  ScalarFunctionQuadraticApproximation cost;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = GetParameters(time, targetTrajectories, preComputation);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  cost.f = adInterfacePtr_->getFunctionValue(tapedTimeStateInput, params)(0);

  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeStateInput, params);
  cost.dfdx = J.middleCols(1, stateDim).transpose();
  cost.dfdu = J.rightCols(inputDim).transpose();

  const matrix_t H = adInterfacePtr_->getHessian(0, tapedTimeStateInput, params);
  cost.dfdxx = H.block(1, 1, stateDim, stateDim);
  cost.dfdux = H.block(1 + stateDim, 1, inputDim, stateDim);
  cost.dfduu = H.bottomRightCorner(inputDim, inputDim);

  return cost;
}

}  // namespace bolza
