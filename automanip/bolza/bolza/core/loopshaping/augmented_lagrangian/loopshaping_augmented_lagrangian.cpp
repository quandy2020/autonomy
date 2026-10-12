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

#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian.hpp>
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian_output_pattern.hpp>
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_state_augmented_lagrangian.hpp>
#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_state_input_augmented_lagrangian.hpp>

namespace bolza {
namespace LoopshapingAugmentedLagrangian {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateAugmentedLagrangianCollection> Create(const StateAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                           std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  return std::make_unique<LoopshapingStateAugmentedLagrangian>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputAugmentedLagrangianCollection> Create(const StateInputAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->GetType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingAugmentedLagrangianOutputPattern>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingAugmentedLagrangianEliminatePattern>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
    default:
      throw std::runtime_error("[LoopshapingAugmentedLagrangian::Create] invalid loopshaping type");
  }
}

}  // namespace LoopshapingAugmentedLagrangian
}  // namespace bolza
