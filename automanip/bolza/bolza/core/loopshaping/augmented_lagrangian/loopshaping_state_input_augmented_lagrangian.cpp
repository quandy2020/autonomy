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
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_state_input_augmented_lagrangian.hpp>

namespace bolza {

std::vector<LagrangianMetrics> LoopshapingStateInputAugmentedLagrangian::GetValue(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                  const std::vector<Multiplier>& termsMultiplier,
                                                                                  const PreComputation& preComp) const {
  if (this->empty()) {
    return {};
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto& preComp_system = preCompLS.GetSystemPreComputation();

  return StateInputAugmentedLagrangianCollection::GetValue(t, x_system, u_system, termsMultiplier, preComp_system);
}

void LoopshapingStateInputAugmentedLagrangian::UpdateLagrangian(scalar_t t, const vector_t& x, const vector_t& u,
                                                                std::vector<LagrangianMetrics>& termsMetrics,
                                                                std::vector<Multiplier>& termsMultiplier) const {
  if (this->empty()) {
    termsMultiplier.clear();
  }

  const auto x_system = loopshapingDefinition_->GetSystemState(x);
  const auto u_system = loopshapingDefinition_->GetSystemInput(x, u);

  StateInputAugmentedLagrangianCollection::UpdateLagrangian(t, x_system, u_system, termsMetrics, termsMultiplier);
}

}  // namespace bolza
