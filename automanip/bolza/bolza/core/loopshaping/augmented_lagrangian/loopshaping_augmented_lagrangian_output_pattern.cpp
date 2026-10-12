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
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian_output_pattern.hpp>

namespace bolza {

ScalarFunctionQuadraticApproximation LoopshapingAugmentedLagrangianOutputPattern::GetQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  // Not const, so we can move
  auto L_system =
      LoopshapingStateInputAugmentedLagrangian::GetQuadraticApproximation(t, x_system, u_system, termsMultiplier, preComp_system);

  ScalarFunctionQuadraticApproximation L;
  L.f = std::move(L_system.f);

  L.dfdx.setZero(stateDim);
  L.dfdx.head(sysStateDim) = L_system.dfdx;

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
