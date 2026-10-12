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
#include <bolza/core/loopshaping/cost/loopshaping_state_input_cost.hpp>

namespace bolza {

scalar_t LoopshapingStateInputCost::GetValue(scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories,
                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return 0.0;
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto& u_filter = preCompLS.GetFilteredInput();

  const scalar_t L_system =
      StateInputCostCollection::GetValue(t, x_system, u_system, targetTrajectories, preCompLS.GetSystemPreComputation());

  return L_system + loopshapingDefinition_->LoopshapingCost(u_filter);
}

}  // namespace bolza
