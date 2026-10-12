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

#include "bolza/tools/common/loopshaping_robot_interface.hpp"

#include <bolza/oc/oc_problem/loopshaping_optimal_control_problem.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingRobotInterface::LoopshapingRobotInterface(std::unique_ptr<RobotInterface> robotInterfacePtr,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr)
    : robotInterfacePtr_(std::move(robotInterfacePtr)), loopshapingDefinitionPtr_(std::move(loopshapingDefinitionPtr)) {
  // wrap with loopshaping
  optimalControlProblem_ =
      LoopshapingOptimalControlProblem::Create(robotInterfacePtr_->getOptimalControlProblem(), loopshapingDefinitionPtr_);

  initializerPtr_.reset(new bolza::LoopshapingInitializer(robotInterfacePtr_->getInitializer(), loopshapingDefinitionPtr_));

  if (robotInterfacePtr_->getReferenceManagerPtr() != nullptr) {
    loopshapingReferenceManager_ =
        std::make_shared<LoopshapingReferenceManager>(robotInterfacePtr_->getReferenceManagerPtr(), loopshapingDefinitionPtr_);
  }
}

}  // namespace bolza
