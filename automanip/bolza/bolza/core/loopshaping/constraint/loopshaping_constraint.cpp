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

#include <bolza/core/loopshaping/constraint/loopshaping_constraint.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_constraint_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_constraint_output_pattern.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_state_constraint.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_state_input_constraint.hpp>

namespace bolza {
namespace LoopshapingConstraint {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateConstraintCollection> Create(const StateConstraintCollection& systemConstraint,
                                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  return std::make_unique<LoopshapingStateConstraint>(systemConstraint, std::move(loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraintCollection> Create(const StateInputConstraintCollection& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingConstraintOutputPattern>(systemConstraint, std::move(loopshapingDefinition));
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingConstraintEliminatePattern>(systemConstraint, std::move(loopshapingDefinition));
    default:
      throw std::runtime_error("[LoopshapingConstraint::Create] invalid loopshaping type");
  }
}

}  // namespace LoopshapingConstraint
}  // namespace bolza
