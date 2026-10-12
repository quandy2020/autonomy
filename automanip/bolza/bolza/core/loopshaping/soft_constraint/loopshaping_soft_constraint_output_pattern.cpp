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

#include <bolza/core/loopshaping/loopshaping_pre_computation.hpp>
#include <bolza/core/loopshaping/soft_constraint/loopshaping_soft_constraint_output_pattern.hpp>

namespace bolza {

ScalarFunctionQuadraticApproximation LoopshapingSoftConstraintOutputPattern::GetQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& r_filter = loopshapingDefinition_->GetInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& x_filter = preCompLS.GetFilterState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  // Not const, so we can move
  auto L_system =
      StateInputCostCollection::GetQuadraticApproximation(t, x_system, u_system, targetTrajectories, preCompLS.GetSystemPreComputation());

  ScalarFunctionQuadraticApproximation L;
  L.f = L_system.f;

  L.dfdx.resize(stateDim);
  L.dfdx.head(sysStateDim) = L_system.dfdx;
  L.dfdx.tail(filtStateDim).setZero();

  L.dfdxx.setZero(stateDim, stateDim);
  L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;

  L.dfdu = std::move(L_system.dfdu);
  L.dfduu = std::move(L_system.dfduu);

  L.dfdux.resize(inputDim, stateDim);
  L.dfdux.leftCols(sysStateDim) = L_system.dfdux;
  L.dfdux.rightCols(filtStateDim).setZero();

  return L;
}

}  // namespace bolza
