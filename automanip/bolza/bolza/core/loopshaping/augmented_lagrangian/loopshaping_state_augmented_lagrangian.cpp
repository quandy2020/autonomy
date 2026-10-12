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
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_state_augmented_lagrangian.hpp>

namespace bolza {

std::vector<LagrangianMetrics> LoopshapingStateAugmentedLagrangian::GetValue(scalar_t t, const vector_t& x,
                                                                             const std::vector<Multiplier>& termsMultiplier,
                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return {};
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();

  return StateAugmentedLagrangianCollection::GetValue(t, x_system, termsMultiplier, preComp_system);
}

ScalarFunctionQuadraticApproximation LoopshapingStateAugmentedLagrangian::GetQuadraticApproximation(
    scalar_t t, const vector_t& x, const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  auto Phi_system = StateAugmentedLagrangianCollection::GetQuadraticApproximation(t, x_system, termsMultiplier, preComp_system);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = std::move(Phi_system.f);
  Phi.dfdx.setZero(stateDim);
  Phi.dfdx.head(sysStateDim) = Phi_system.dfdx;
  Phi.dfdxx.setZero(stateDim, stateDim);
  Phi.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = Phi_system.dfdxx;
  return Phi;
}

void LoopshapingStateAugmentedLagrangian::UpdateLagrangian(scalar_t t, const vector_t& x, std::vector<LagrangianMetrics>& termsMetrics,
                                                           std::vector<Multiplier>& termsMultiplier) const {
  if (this->empty()) {
    termsMultiplier.clear();
  }

  const auto x_system = loopshapingDefinition_->getSystemState(x);
  StateAugmentedLagrangianCollection::UpdateLagrangian(t, x_system, termsMetrics, termsMultiplier);
}

}  // namespace bolza
