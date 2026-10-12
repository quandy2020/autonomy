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

#include <bolza/core/cost/quadratic_state_input_cost.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateInputCost::QuadraticStateInputCost(matrix_t Q, matrix_t R, matrix_t P /* = matrix_t() */)
    : Q_(std::move(Q)), R_(std::move(R)), P_(std::move(P)) {
  if (P_.size() > 0) {
    assert(P_.rows() == R_.rows());
    assert(P_.cols() == Q_.rows());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateInputCost* QuadraticStateInputCost::Clone() const {
  return new QuadraticStateInputCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticStateInputCost::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                           const TargetTrajectories& targetTrajectories, const PreComputation&) const {
  vector_t stateDeviation, inputDeviation;
  std::tie(stateDeviation, inputDeviation) = GetStateInputDeviation(time, state, input, targetTrajectories);

  if (P_.size() == 0) {
    return 0.5 * stateDeviation.dot(Q_ * stateDeviation) + 0.5 * inputDeviation.dot(R_ * inputDeviation);
  } else {
    return 0.5 * stateDeviation.dot(Q_ * stateDeviation) + 0.5 * inputDeviation.dot(R_ * inputDeviation) +
           inputDeviation.dot(P_ * stateDeviation);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticStateInputCost::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const TargetTrajectories& targetTrajectories,
                                                                                        const PreComputation&) const {
  vector_t stateDeviation, inputDeviation;
  std::tie(stateDeviation, inputDeviation) = GetStateInputDeviation(time, state, input, targetTrajectories);

  ScalarFunctionQuadraticApproximation L;
  L.dfdxx = Q_;
  L.dfduu = R_;
  L.dfdx.noalias() = Q_ * stateDeviation;
  L.dfdu.noalias() = R_ * inputDeviation;
  L.f = 0.5 * stateDeviation.dot(L.dfdx) + 0.5 * inputDeviation.dot(L.dfdu);

  if (P_.size() == 0) {
    L.dfdux.setZero(input.size(), state.size());

  } else {
    const vector_t pDeviation = P_ * stateDeviation;
    L.f += inputDeviation.dot(pDeviation);
    L.dfdu += pDeviation;
    L.dfdx.noalias() += P_.transpose() * inputDeviation;
    L.dfdux = P_;
  }

  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> QuadraticStateInputCost::GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const TargetTrajectories& targetTrajectories) const {
  const vector_t stateDeviation = state - targetTrajectories.GetDesiredState(time);
  const vector_t inputDeviation = input - targetTrajectories.GetDesiredInput(time);
  return {stateDeviation, inputDeviation};
}

}  // namespace bolza
