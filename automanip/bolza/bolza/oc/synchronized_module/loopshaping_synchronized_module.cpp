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

#include "bolza/oc/synchronized_module/loopshaping_synchronized_module.hpp"

#include "bolza/oc/oc_data/loopshaping_primal_solution.hpp"

namespace bolza {

LoopshapingSynchronizedModule::LoopshapingSynchronizedModule(
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr,
    std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulesPtrArray)
    : loopshapingDefinitionPtr_(std::move(loopshapingDefinitionPtr)),
      synchronizedModulesPtrArray_(std::move(synchronizedModulesPtrArray)) {}

void LoopshapingSynchronizedModule::PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                 const ReferenceManagerInterface& referenceManager) {
  if (!synchronizedModulesPtrArray_.empty()) {
    const auto systemState = loopshapingDefinitionPtr_->GetSystemState(initState);
    for (auto& module : synchronizedModulesPtrArray_) {
      module->PreSolverRun(initTime, finalTime, systemState, referenceManager);
    }
  }
}

void LoopshapingSynchronizedModule::PostSolverRun(const PrimalSolution& primalSolution) {
  if (!synchronizedModulesPtrArray_.empty()) {
    const auto systemPrimalSolution = LoopshapingToSystemPrimalSolution(primalSolution, *loopshapingDefinitionPtr_);

    for (auto& module : synchronizedModulesPtrArray_) {
      module->PostSolverRun(systemPrimalSolution);
    }
  }
}

}  // namespace bolza
