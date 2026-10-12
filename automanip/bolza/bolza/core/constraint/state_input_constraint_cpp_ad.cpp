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

#include <bolza/core/constraint/state_input_constraint_cpp_ad.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputConstraintCppAd::Initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                           const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto constraintAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->ConstraintFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new bolza::CppAdInterface(constraintAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

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
StateInputConstraintCppAd::StateInputConstraintCppAd(const StateInputConstraintCppAd& rhs)
    : StateInputConstraint(rhs), adInterfacePtr_(new bolza::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateInputConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                             const PreComputation& preComputation) const {
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  tapedTimeStateInput << time, state, input;
  return adInterfacePtr_->GetFunctionValue(tapedTimeStateInput, GetParameters(time, preComputation));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateInputConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input,
                                                                                    const PreComputation& preComputation) const {
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = GetParameters(time, preComputation);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  constraint.f = adInterfacePtr_->GetFunctionValue(tapedTimeStateInput, params);
  const matrix_t J = adInterfacePtr_->GetJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateInputConstraintCppAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComputation) const {
  if (GetOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateInputConstraintCppAd] Quadratic approximation not supported!");
  }

  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = GetParameters(time, preComputation);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  constraint.f = adInterfacePtr_->GetFunctionValue(tapedTimeStateInput, params);
  const matrix_t J = adInterfacePtr_->GetJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = adInterfacePtr_->GetHessian(i, tapedTimeStateInput, params);
    constraint.dfdxx[i] = H.block(1, 1, stateDim, stateDim);
    constraint.dfdux[i] = H.block(1 + stateDim, 1, inputDim, stateDim);
    constraint.dfduu[i] = H.bottomRightCorner(inputDim, inputDim);
  }

  return constraint;
}

}  // namespace bolza
