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

#include "bolza/core/loopshaping/initialization/loopshaping_initializer.hpp"

#include <iostream>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer::LoopshapingInitializer(const Initializer& systembase, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
    : systembase_(systembase.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer::LoopshapingInitializer(const LoopshapingInitializer& other)
    : Initializer(other), systembase_(other.systembase_->clone()), loopshapingDefinition_(other.loopshapingDefinition_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingInitializer* LoopshapingInitializer::Clone() const {
  return new LoopshapingInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  // system state-input initializer
  vector_t systemInput, systemNextState;
  const vector_t systemState = loopshapingDefinition_->getSystemState(state);
  systembase_->compute(time, systemState, nextTime, systemInput, systemNextState);

  // Compute input
  vector_t equilibriumFilterInput;
  const vector_t filterState = loopshapingDefinition_->getFilterState(state);
  loopshapingDefinition_->getFilterEquilibriumGivenState(systemInput, filterState, equilibriumFilterInput);
  input = loopshapingDefinition_->augmentedSystemInput(systemInput, equilibriumFilterInput);

  // Next filter state
  vector_t filterNextState = filterState + (nextTime - time) * loopshapingDefinition_->filterFlowMap(filterState, input);
  nextState = loopshapingDefinition_->concatenateSystemAndFilterState(systemNextState, filterNextState);
}

}  // namespace bolza
