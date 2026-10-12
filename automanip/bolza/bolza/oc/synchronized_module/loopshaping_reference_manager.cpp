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

#include "bolza/oc/synchronized_module/loopshaping_reference_manager.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingReferenceManager::LoopshapingReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                                                         std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)), loopshapingDefinitionPtr_(std::move(loopshapingDefinitionPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LoopshapingReferenceManager::PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) {
  const vector_t systemState = loopshapingDefinitionPtr_->getSystemState(initState);
  referenceManagerPtr_->preSolverRun(initTime, finalTime, systemState);
};

}  // namespace bolza
