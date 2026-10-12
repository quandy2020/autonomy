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

#include <bolza/core/constraint/state_constraint_cpp_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateConstraintCppAd::Initialize(size_t stateDim, size_t parameterDim, const std::string& modelName, const std::string& modelFolder,
                                      bool recompileLibraries, bool verbose) {
  auto constraintAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->ConstraintFunction(time, state, p);
  };
  adInterfacePtr_.reset(new bolza::CppAdInterface(constraintAd, 1 + stateDim, parameterDim, modelName, modelFolder));

  bolza::CppAdInterface::ApproximationOrder orderCppAd;
  if (GetOrder() == ConstraintOrder::Linear) {
    orderCppAd = bolza::CppAdInterface::ApproximationOrder::First;
  } else {
    orderCppAd = bolza::CppAdInterface::ApproximationOrder::Second;
  }

  if (recompileLibraries) {
    adInterfacePtr_->CreateModels(orderCppAd, verbose);
  } else {
    adInterfacePtr_->LoadModelsIfAvailable(orderCppAd, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateConstraintCppAd::StateConstraintCppAd(const StateConstraintCppAd& rhs)
    : StateConstraint(rhs), adInterfacePtr_(new bolza::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;
  return adInterfacePtr_->GetFunctionValue(tapedTimeState, GetParameters(time, preComputation));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                               const PreComputation& preComputation) const {
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = GetParameters(time, preComputation);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  constraint.f = adInterfacePtr_->GetFunctionValue(tapedTimeState, params);
  const matrix_t J = adInterfacePtr_->GetJacobian(tapedTimeState, params);
  constraint.dfdx = J.rightCols(stateDim);

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateConstraintCppAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                     const PreComputation& preComputation) const {
  if (GetOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateConstraintCppAd] Quadratic approximation not supported!");
  }

  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = GetParameters(time, preComputation);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  constraint.f = adInterfacePtr_->GetFunctionValue(tapedTimeState, params);
  const matrix_t J = adInterfacePtr_->GetJacobian(tapedTimeState, params);
  constraint.dfdx = J.rightCols(stateDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = adInterfacePtr_->GetHessian(i, tapedTimeState, params);
    constraint.dfdxx[i] = H.bottomRightCorner(stateDim, stateDim);
  }

  return constraint;
}

}  // namespace bolza
