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

#include "bolza/oc/rollout/time_triggered_rollout.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TimeTriggeredRollout::TimeTriggeredRollout(const ControlledSystemBase& systemDynamics, rollout::Settings rolloutSettings)
    : RolloutBase(std::move(rolloutSettings)), systemDynamicsPtr_(systemDynamics.Clone()), systemEventHandlersPtr_(new SystemEventHandler) {
  // construct dynamicsIntegratorsPtr
  dynamicsIntegratorPtr_ = std::move(NewIntegrator(this->Settings().integratorType, systemEventHandlersPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TimeTriggeredRollout::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                                   ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                                   vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  if (initTime > finalTime) {
    throw std::runtime_error("[TimeTriggeredRollout::Run] The initial time should be less-equal to the final time!");
  }
  if (controller == nullptr) {
    throw std::runtime_error("[TimeTriggeredRollout::Run] Controller is not set!");
  }

  // extract sub-systems
  const auto timeIntervalArray = FindActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const int numSubsystems = timeIntervalArray.size();
  const int numEvents = numSubsystems - 1;

  // max number of steps for integration
  const auto maxNumSteps = static_cast<size_t>(this->Settings().maxNumStepsPerSecond * std::max(1.0, finalTime - initTime));

  // clearing the output trajectories
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 1);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 1);
  postEventIndices.clear();
  postEventIndices.reserve(numEvents);

  // set controller
  systemDynamicsPtr_->SetController(controller);

  // reset function calls counter
  systemDynamicsPtr_->ResetNumFunctionCalls();

  // reset the event class
  systemEventHandlersPtr_->Reset();

  vector_t beginState = initState;
  int k_u = 0;  // control input iterator
  for (int i = 0; i < numSubsystems; i++) {
    if (timeIntervalArray[i].first < timeIntervalArray[i].second) {
      Observer observer(&stateTrajectory, &timeTrajectory);  // concatenate trajectory
      // integrate controlled system
      dynamicsIntegratorPtr_->IntegrateAdaptive(*systemDynamicsPtr_, observer, beginState, timeIntervalArray[i].first,
                                                timeIntervalArray[i].second, this->Settings().timeStep, this->Settings().absTolODE,
                                                this->Settings().relTolODE, maxNumSteps);
    } else {
      timeTrajectory.push_back(timeIntervalArray[i].second);
      stateTrajectory.push_back(beginState);
    }

    // compute control input trajectory and concatenate to inputTrajectory
    if (this->Settings().reconstructInputTrajectory) {
      for (; k_u < timeTrajectory.size(); k_u++) {
        inputTrajectory.emplace_back(systemDynamicsPtr_->ControllerPtr()->ComputeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
      }  // end of k_u loop
    }

    // a jump has taken place
    if (i < numEvents) {
      postEventIndices.push_back(stateTrajectory.size());
      // jump map
      beginState = systemDynamicsPtr_->ComputeJumpMap(timeTrajectory.back(), stateTrajectory.back());
    }
  }  // end of i loop

  // check for the numerical stability
  this->CheckNumericalStability(*controller, timeTrajectory, postEventIndices, stateTrajectory, inputTrajectory);

  return stateTrajectory.back();
}

}  // namespace bolza
