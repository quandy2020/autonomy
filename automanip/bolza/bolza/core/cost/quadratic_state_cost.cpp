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

#include <bolza/core/cost/quadratic_state_cost.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateCost::QuadraticStateCost(matrix_t Q) : Q_(std::move(Q)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateCost* QuadraticStateCost::Clone() const {
  return new QuadraticStateCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticStateCost::GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                                      const PreComputation&) const {
  const vector_t xDeviation = GetStateDeviation(time, state, targetTrajectories);
  return 0.5 * xDeviation.dot(Q_ * xDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticStateCost::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                   const TargetTrajectories& targetTrajectories,
                                                                                   const PreComputation&) const {
  const vector_t xDeviation = GetStateDeviation(time, state, targetTrajectories);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.dfdxx = Q_;
  Phi.dfdx.noalias() = Q_ * xDeviation;
  Phi.f = 0.5 * xDeviation.dot(Phi.dfdx);
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticStateCost::GetStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const {
  return state - targetTrajectories.GetDesiredState(time);
}

}  // namespace bolza
