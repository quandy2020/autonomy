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

#include "examples/legged_robot/gait/gait_schedule.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
    : modeSchedule_(std::move(initModeSchedule)),
      modeSequenceTemplate_(std::move(initModeSequenceTemplate)),
      phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::InsertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime) {
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (Index < eventTimes.size()) {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE) {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0) {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  TileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::GetModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

    // set the default initial phase
    if (!modeSequence.empty()) {
      modeSequence.front() = ModeNumber::STANCE;
    }
  }

  // Start tiling at time. An empty schedule has no tail phase to delete;
  // erasing end()-1 there is undefined and aborts the process.
  if (eventTimes.empty() || modeSequence.empty()) {
    eventTimes.clear();
    modeSequence.clear();
    TileModeSequenceTemplate(lowerBoundTime, upperBoundTime);
    return modeSchedule_;
  }
  const auto tilingStartTime = eventTimes.back();

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

  // tile the template logic
  TileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::TileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // concatenate from index. A non-positive phase length never reaches finalTime
  // and the vector grows until the process is killed with std::bad_alloc.
  while (eventTimes.back() < finalTime && eventTimes.size() < 100000) {
    bool stop = false;
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      if (i + 1 >= templateTimes.size()) {
        stop = true;
        break;
      }
      const scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
      if (!(deltaTime > 0.0)) {
        stop = true;
        break;
      }
      modeSequence.push_back(templateModeSequence[i]);
      eventTimes.push_back(eventTimes.back() + deltaTime);
    }  // end of i loop
    if (stop) {
      break;
    }
  }  // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);
}

}  // namespace legged_robot
}  // namespace bolza
