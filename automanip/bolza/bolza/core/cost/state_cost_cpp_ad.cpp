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

#include <bolza/core/cost/state_cost_cpp_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateCostCppAd::Initialize(size_t stateDim, size_t parameterDim, const std::string& modelName, const std::string& modelFolder,
                                bool recompileLibraries, bool verbose) {
  auto costAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = ad_vector_t(1);
    y(0) = this->costFunction(time, state, p);
  };
  adInterfacePtr_.reset(new bolza::CppAdInterface(costAd, 1 + stateDim, parameterDim, modelName, modelFolder));

  if (recompileLibraries) {
    adInterfacePtr_->createModels(bolza::CppAdInterface::ApproximationOrder::Second, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(bolza::CppAdInterface::ApproximationOrder::Second, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCppAd::StateCostCppAd(const StateCostCppAd& rhs)
    : StateCost(rhs), adInterfacePtr_(new bolza::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateCostCppAd::GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                                  const PreComputation& preComputation) const {
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;
  return adInterfacePtr_->getFunctionValue(tapedTimeState, GetParameters(time, targetTrajectories, preComputation))(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateCostCppAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                               const TargetTrajectories& targetTrajectories,
                                                                               const PreComputation& preComputation) const {
  ScalarFunctionQuadraticApproximation cost;

  const size_t stateDim = state.rows();
  const vector_t params = GetParameters(time, targetTrajectories, preComputation);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  cost.f = adInterfacePtr_->getFunctionValue(tapedTimeState, params)(0);

  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeState, params);
  cost.dfdx = J.rightCols(stateDim).transpose();

  const matrix_t H = adInterfacePtr_->getHessian(0, tapedTimeState, params);
  cost.dfdxx = H.bottomRightCorner(stateDim, stateDim);

  return cost;
}

}  // namespace bolza
