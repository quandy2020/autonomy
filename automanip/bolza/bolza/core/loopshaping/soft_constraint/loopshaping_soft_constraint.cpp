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

#include <bolza/core/loopshaping/cost/loopshaping_state_cost.hpp>
#include <bolza/core/loopshaping/soft_constraint/loopshaping_soft_constraint.hpp>
#include <bolza/core/loopshaping/soft_constraint/loopshaping_soft_constraint_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/soft_constraint/loopshaping_soft_constraint_output_pattern.hpp>
#include <bolza/core/loopshaping/soft_constraint/loopshaping_state_input_soft_constraint.hpp>

namespace bolza {
namespace LoopshapingSoftConstraint {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCostCollection> Create(const StateCostCollection& systemSoftConstraint,
                                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  // State-only soft constraint wrapper is identical to LoopshapingStateCost wrapper.
  return std::make_unique<LoopshapingStateCost>(systemSoftConstraint, std::move(loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCostCollection> Create(const StateInputCostCollection& systemSoftConstraint,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->GetType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingSoftConstraintOutputPattern>(systemSoftConstraint, std::move(loopshapingDefinition));
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingSoftConstraintEliminatePattern>(systemSoftConstraint, std::move(loopshapingDefinition));
    default:
      throw std::runtime_error("[LoopshapingStateInputSoftConstraint::Create] invalid loopshaping type");
  }
}

}  // namespace LoopshapingSoftConstraint
}  // namespace bolza
