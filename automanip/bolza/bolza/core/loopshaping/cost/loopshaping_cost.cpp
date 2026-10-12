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

#include <bolza/core/loopshaping/cost/loopshaping_cost.hpp>
#include <bolza/core/loopshaping/cost/loopshaping_cost_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/cost/loopshaping_cost_output_pattern.hpp>
#include <bolza/core/loopshaping/cost/loopshaping_state_cost.hpp>
#include <bolza/core/loopshaping/cost/loopshaping_state_input_cost.hpp>

namespace bolza {
namespace LoopshapingCost {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCostCollection> Create(const StateCostCollection& systemCost,
                                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  return std::make_unique<LoopshapingStateCost>(systemCost, std::move(loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCostCollection> Create(const StateInputCostCollection& systemCost,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingCostOutputPattern>(systemCost, std::move(loopshapingDefinition));
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingCostEliminatePattern>(systemCost, std::move(loopshapingDefinition));
    default:
      throw std::runtime_error("[LoopshapingStateInputCost::Create] invalid loopshaping type");
  }
}

}  // namespace LoopshapingCost
}  // namespace bolza
