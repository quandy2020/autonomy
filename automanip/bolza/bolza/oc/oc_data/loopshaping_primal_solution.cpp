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

#include "bolza/oc/oc_data/loopshaping_primal_solution.hpp"

namespace bolza {

PrimalSolution LoopshapingToSystemPrimalSolution(const PrimalSolution& primalSolution, const LoopshapingDefinition& loopshapingDefinition) {
  PrimalSolution systemPrimalSolution;
  systemPrimalSolution.controllerPtr_ = nullptr;
  systemPrimalSolution.timeTrajectory_ = primalSolution.timeTrajectory_;
  systemPrimalSolution.modeSchedule_ = primalSolution.modeSchedule_;
  systemPrimalSolution.stateTrajectory_.reserve(primalSolution.stateTrajectory_.size());
  systemPrimalSolution.inputTrajectory_.reserve(primalSolution.inputTrajectory_.size());
  for (size_t k = 0; k < primalSolution.stateTrajectory_.size(); ++k) {
    systemPrimalSolution.stateTrajectory_.push_back(loopshapingDefinition.getSystemState(primalSolution.stateTrajectory_[k]));
    systemPrimalSolution.inputTrajectory_.push_back(
        loopshapingDefinition.getSystemInput(primalSolution.stateTrajectory_[k], primalSolution.inputTrajectory_[k]));
  }
  return systemPrimalSolution;
}

}  // namespace bolza
