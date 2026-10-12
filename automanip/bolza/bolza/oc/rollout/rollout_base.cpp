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

#include "bolza/oc/rollout/rollout_base.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/misc/numerics.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<scalar_t, scalar_t>> RolloutBase::FindActiveModesTimeInterval(scalar_t initTime, scalar_t finalTime,
                                                                                    const scalar_array_t& eventTimes) const {
  // switching times
  const auto firstIndex = std::upper_bound(eventTimes.cbegin(), eventTimes.cend(), initTime);  // no event at initial time
  const auto lastIndex = std::upper_bound(eventTimes.cbegin(), eventTimes.cend(), finalTime);  // can be an event at final time
  scalar_array_t switchingTimes;
  switchingTimes.push_back(initTime);
  switchingTimes.insert(switchingTimes.end(), firstIndex, lastIndex);
  switchingTimes.push_back(finalTime);

  // constructing the rollout time intervals
  const int numSubsystems = switchingTimes.size() - 1;  // switchingTimes contains at least two elements
  std::vector<std::pair<scalar_t, scalar_t>> timeIntervalArray(numSubsystems);
  for (int i = 0; i < numSubsystems; i++) {
    const auto& beginTime = switchingTimes[i];
    const auto& endTime = switchingTimes[i + 1];
    timeIntervalArray[i] = std::make_pair(beginTime, endTime);

    // adjusting the start time to correct for subsystem recognition
    constexpr auto eps = numeric_traits::WeakEpsilon<scalar_t>();
    timeIntervalArray[i].first = std::min(beginTime + eps, endTime);
  }  // end of for loop

  return timeIntervalArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RolloutBase::Display(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, const vector_array_t& stateTrajectory,
                          const vector_array_t* const inputTrajectory) {
  std::cerr << "Trajectory length:      " << timeTrajectory.size() << '\n';
  std::cerr << "Total number of events: " << postEventIndices.size() << '\n';
  if (!postEventIndices.empty()) {
    std::cerr << "Event times: ";
    for (size_t ind : postEventIndices) {
      std::cerr << timeTrajectory[ind] << ", ";
    }
    std::cerr << '\n';
  }
  std::cerr << '\n';

  const size_t numSubsystems = postEventIndices.size() + 1;
  size_t k = 0;
  for (size_t i = 0; i < numSubsystems; i++) {
    for (; k < timeTrajectory.size(); k++) {
      std::cerr << "Index: " << k << '\n';
      std::cerr << "Time:  " << std::setprecision(12) << timeTrajectory[k] << '\n';
      std::cerr << "State: " << std::setprecision(3) << stateTrajectory[k].transpose() << '\n';
      if (inputTrajectory != nullptr) {
        std::cerr << "Input: " << std::setprecision(3) << (*inputTrajectory)[k].transpose() << '\n';
      }

      if (i < postEventIndices.size() && k + 1 == postEventIndices[i]) {
        std::cerr << "+++ event took place +++" << '\n';
        k++;
        break;
      }
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RolloutBase::CheckNumericalStability(const ControllerBase& controller, const scalar_array_t& timeTrajectory,
                                          const size_array_t& postEventIndices, const vector_array_t& stateTrajectory,
                                          const vector_array_t& inputTrajectory) const {
  if (!rolloutSettings_.checkNumericalStability) {
    return;
  }

  for (size_t i = 0; i < timeTrajectory.size(); i++) {
    try {
      if (!stateTrajectory[i].allFinite()) {
        throw std::runtime_error("Rollout: state is not finite");
      }
      if (rolloutSettings_.reconstructInputTrajectory && !inputTrajectory[i].allFinite()) {
        throw std::runtime_error("Rollout: input is not finite");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " + std::to_string(timeTrajectory[i]) + " [sec]." << '\n';

      // truncate trajectories
      scalar_array_t timeTrajectoryTemp;
      vector_array_t stateTrajectoryTemp;
      vector_array_t inputTrajectoryTemp;
      for (size_t j = 0; j <= i; j++) {
        timeTrajectoryTemp.push_back(timeTrajectory[j]);
        stateTrajectoryTemp.push_back(stateTrajectory[j]);
        if (rolloutSettings_.reconstructInputTrajectory) {
          inputTrajectoryTemp.push_back(inputTrajectory[j]);
        }
      }

      // display
      const vector_array_t* const inputTrajectoryTempPtr = rolloutSettings_.reconstructInputTrajectory ? &inputTrajectoryTemp : nullptr;
      Display(timeTrajectoryTemp, postEventIndices, stateTrajectoryTemp, inputTrajectoryTempPtr);

      controller.Display();

      throw;
    }
  }  // end of i loop
}
}  // namespace bolza
