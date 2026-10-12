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

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation::LoopshapingPreComputation(const PreComputation& systemPreComputation,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
    : systemPreCompPtr_(systemPreComputation.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation::LoopshapingPreComputation(const LoopshapingPreComputation& other)
    : loopshapingDefinition_(other.loopshapingDefinition_) {
  systemPreCompPtr_.reset(other.systemPreCompPtr_->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingPreComputation* LoopshapingPreComputation::Clone() const {
  return new LoopshapingPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getSystemInput(x, u, systemInput_);
  loopshapingDefinition_->getFilterState(x, filterState_);
  loopshapingDefinition_->getFilteredInput(x, u, filterInput_);

  systemPreCompPtr_->request(request, t, systemState_, systemInput_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::RequestPreJump(RequestSet request, scalar_t t, const vector_t& x) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getFilterState(x, filterState_);

  systemPreCompPtr_->requestPreJump(request, t, systemState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingPreComputation::RequestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  loopshapingDefinition_->getSystemState(x, systemState_);
  loopshapingDefinition_->getFilterState(x, filterState_);

  systemPreCompPtr_->requestFinal(request, t, systemState_);
}

}  // namespace bolza
