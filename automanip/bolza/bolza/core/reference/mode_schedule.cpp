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

#include "bolza/core/reference/mode_schedule.hpp"

#include <bolza/core/misc/display.hpp>
#include <bolza/core/misc/lookup.hpp>
#include <bolza/core/misc/numerics.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule::ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput)
    : eventTimes(std::move(eventTimesInput)), modeSequence(std::move(modeSequenceInput)) {
  assert(!modeSequence.empty());
  assert(eventTimes.size() + 1 == modeSequence.size());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ModeSchedule::ModeAtTime(scalar_t time) const {
  const auto ind = lookup::FindIndexInTimeArray(eventTimes, time);
  return modeSequence[ind];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(ModeSchedule& lh, ModeSchedule& rh) {
  lh.eventTimes.swap(rh.eventTimes);
  lh.modeSequence.swap(rh.modeSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSchedule& modeSchedule) {
  stream << "event times:   {" << ToDelimitedString(modeSchedule.eventTimes) << "}\n";
  stream << "mode sequence: {" << ToDelimitedString(modeSchedule.modeSequence) << "}\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t GetNumberOfPrecedingEvents(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, scalar_t eventTime) {
  // the case of empty time trajectory and eventTime smaller or equal to the initial time
  if (timeTrajectory.empty() || eventTime <= timeTrajectory.front()) {
    return 0;
  }

  const auto eventIndexItr = std::find_if(postEventIndices.cbegin(), postEventIndices.cend(), [&](size_t postEventInd) {
    return numerics::almost_eq(eventTime, timeTrajectory[postEventInd - 1]);
  });

  // if the given time did not match any event time but it is smaller than the final time
  if (eventIndexItr == postEventIndices.cend() && eventTime < timeTrajectory.back()) {
    throw std::runtime_error(
        "[getNumberOfPrecedingEvents] The requested time is within the time trajectory but it is not marked as an event!");
  }

  return std::distance(postEventIndices.cbegin(), eventIndexItr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, scalar_t> FindIntersectionToExtendableInterval(const scalar_array_t& timeTrajectory, const scalar_array_t& eventTimes,
                                                                   const std::pair<scalar_t, scalar_t>& timePeriod) {
  // no interpolation: a bit before initial time
  const std::pair<scalar_t, scalar_t> emptyInterpolatableInterval{timePeriod.first, timePeriod.first - 1e-4};

  if (timeTrajectory.empty() || timePeriod.first > timePeriod.second) {
    return emptyInterpolatableInterval;

  } else {
    const auto pastEventItr =
        std::find_if(eventTimes.crbegin(), eventTimes.crend(), [&](const scalar_t& te) { return te <= timeTrajectory.front(); });
    const auto initialTime = (pastEventItr != eventTimes.crend()) ? std::max(*pastEventItr, timePeriod.first) : timePeriod.first;

    const auto nextEventItr = std::lower_bound(eventTimes.cbegin(), eventTimes.cend(), timeTrajectory.back());
    const auto finalTime = (nextEventItr != eventTimes.cend()) ? std::min(*nextEventItr, timePeriod.second) : timePeriod.second;

    return (initialTime < finalTime) ? std::make_pair(initialTime, finalTime) : emptyInterpolatableInterval;
  }
}

}  // namespace bolza
