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

#include "bolza/oc/rollout/initializer_rollout.hpp"

#include <bolza/core/numeric_traits.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
InitializerRollout::InitializerRollout(const Initializer& initializer, rollout::Settings rolloutSettings /* = rollout::Settings() */)
    : RolloutBase(std::move(rolloutSettings)), initializerPtr_(initializer.Clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
InitializerRollout* InitializerRollout::Clone() const {
  return new InitializerRollout(*initializerPtr_, this->Settings());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t InitializerRollout::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                                 ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                                 vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  if (initTime > finalTime) {
    throw std::runtime_error("[InitializerRollout::Run] The initial time should be less-equal to the final time!");
  }

  // extract sub-systems
  const auto timeIntervalArray = FindActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const auto numSubsystems = timeIntervalArray.size();
  const auto numEvents = numSubsystems - 1;
  const size_t maxNumSteps = (timeIntervalArray.back().second - timeIntervalArray.front().first) / Settings().timeStep;

  // clearing the output trajectories
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  postEventIndices.clear();
  postEventIndices.reserve(numEvents);

  vector_t input;
  vector_t state = initState;
  for (size_t i = 0; i < numSubsystems; i++) {
    const size_t numSteps = (timeIntervalArray[i].second - timeIntervalArray[i].first) / Settings().timeStep;
    const scalar_t remainderTime = timeIntervalArray[i].second - (timeIntervalArray[i].first + numSteps * Settings().timeStep);

    // take (numSteps + 1) steps from timeIntervalArray[i].first to (timeIntervalArray[i].second - remainderTime)
    for (size_t k = 0; k < numSteps + 1; k++) {
      timeTrajectory.push_back(timeIntervalArray[i].first + k * Settings().timeStep);
      stateTrajectory.push_back(state);
      const scalar_t timeStep = k < numSteps ? Settings().timeStep : remainderTime;
      initializerPtr_->compute(timeTrajectory.back(), stateTrajectory.back(), timeTrajectory.back() + timeStep, input, state);
      inputTrajectory.push_back(input);
    }  // end of k loop

    // if the remainder time is not very small push a new entry otherwise modify the last time
    if (remainderTime > 10.0 * numeric_traits::LimitEpsilon<scalar_t>()) {
      timeTrajectory.push_back(timeIntervalArray[i].second);
      stateTrajectory.push_back(state);
      inputTrajectory.push_back(input);
    } else {
      timeTrajectory.back() = timeIntervalArray[i].second;
    }

    if (i < numEvents) {
      postEventIndices.push_back(stateTrajectory.size());
    }
  }  // end of i loop

  return stateTrajectory.back();
}

}  // namespace bolza
