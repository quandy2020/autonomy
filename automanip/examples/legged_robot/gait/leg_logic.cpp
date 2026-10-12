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

#include "examples/legged_robot/gait/leg_logic.hpp"

#include "examples/legged_robot/gait/motion_phase_definition.hpp"

namespace {

inline bolza::scalar_t TimingNaN() {
  return std::numeric_limits<bolza::scalar_t>::quiet_NaN();
}

inline bool HasStartTime(const bolza::legged_robot::ContactTiming& timing) {
  return !std::Isnan(timing.start);
}
inline bool HasEndTime(const bolza::legged_robot::ContactTiming& timing) {
  return !std::Isnan(timing.end);
}

inline bool HasStartTime(const bolza::legged_robot::SwingTiming& timing) {
  return !std::Isnan(timing.start);
}
inline bool HasEndTime(const bolza::legged_robot::SwingTiming& timing) {
  return !std::Isnan(timing.end);
}

inline bool StartsWithSwingPhase(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return timings.empty() || HasStartTime(timings.front());
}
inline bool startsWithContactPhase(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return !StartsWithSwingPhase(timings);
}
inline bool EndsWithSwingPhase(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return timings.empty() || HasEndTime(timings.back());
}
inline bool endsWithContactPhase(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return !EndsWithSwingPhase(timings);
}

inline bool startsWithContactPhase(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return timings.empty() || HasStartTime(timings.front());
}
inline bool StartsWithSwingPhase(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return !startsWithContactPhase(timings);
}
inline bool endsWithContactPhase(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return timings.empty() || HasEndTime(timings.back());
}
inline bool EndsWithSwingPhase(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return !endsWithContactPhase(timings);
}

inline bool TouchesDownAtLeastOnce(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const bolza::legged_robot::ContactTiming& timing) { return HasStartTime(timing); });
}

inline bool LiftsOffAtLeastOnce(const std::vector<bolza::legged_robot::ContactTiming>& timings) {
  return !timings.empty() && HasEndTime(timings.front());
}

inline bool TouchesDownAtLeastOnce(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return !timings.empty() && HasEndTime(timings.front());
}

inline bool LiftsOffAtLeastOnce(const std::vector<bolza::legged_robot::SwingTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const bolza::legged_robot::SwingTiming& timing) { return HasStartTime(timing); });
}

}  // anonymous namespace

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<LegPhase> GetContactPhasePerLeg(scalar_t time, const bolza::ModeSchedule& modeSchedule) {
  feet_array_t<LegPhase> contactPhasePerLeg;

  // Convert mode sequence to a contact timing vector per leg
  const auto contactTimingsPerLeg = ExtractContactTimingsPerLeg(modeSchedule);

  // Extract contact phases per leg
  for (size_t leg = 0; leg < contactPhasePerLeg.size(); ++leg) {
    if (contactTimingsPerLeg[leg].empty()) {
      // Leg is always in swing phase
      contactPhasePerLeg[leg].phase = -1.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
    } else if (startsWithContactPhase(contactTimingsPerLeg[leg]) && (Time <= contactTimingsPerLeg[leg].front().end)) {
      // It is assumed that contact phase started at minus infinity, so current time will be always close to ContactTiming.end
      contactPhasePerLeg[leg].phase = 1.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else if (endsWithContactPhase(contactTimingsPerLeg[leg]) && (time >= contactTimingsPerLeg[leg].back().start)) {
      // It is assumed that contact phase ends at infinity, so current time will be always close to ContactTiming.start
      contactPhasePerLeg[leg].phase = 0.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else {
      // Check if leg is in contact interval at current time
      auto it = std::find_if(contactTimingsPerLeg[leg].begin(), contactTimingsPerLeg[leg].end(),
                             [time](ContactTiming timing) { return (timing.start <= time) && (Time <= timing.end); });
      if (it == contactTimingsPerLeg[leg].end()) {
        // Leg is not in contact for current time
        contactPhasePerLeg[leg].phase = -1.0;
        contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
      } else {
        // Leg is in contact for current time
        const auto& currentContactTiming = *it;
        contactPhasePerLeg[leg].phase = (time - currentContactTiming.start) / (currentContactTiming.end - currentContactTiming.start);
        contactPhasePerLeg[leg].duration = currentContactTiming.end - currentContactTiming.start;
      }
    }
  }

  return contactPhasePerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<LegPhase> GetSwingPhasePerLeg(scalar_t time, const bolza::ModeSchedule& modeSchedule) {
  feet_array_t<LegPhase> swingPhasePerLeg;

  // Convert mode sequence to a swing timing vector per leg
  const auto swingTimingsPerLeg = ExtractSwingTimingsPerLeg(modeSchedule);

  // Extract swing phases per leg
  for (size_t leg = 0; leg < swingPhasePerLeg.size(); ++leg) {
    if (swingTimingsPerLeg[leg].empty()) {
      // Leg is always in contact phase
      swingPhasePerLeg[leg].phase = -1.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
    } else if (StartsWithSwingPhase(swingTimingsPerLeg[leg]) && (Time <= swingTimingsPerLeg[leg].front().end)) {
      // It is assumed that swing phase started at minus infinity, so current time will be always close to SwingTiming.end
      swingPhasePerLeg[leg].phase = 1.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else if (EndsWithSwingPhase(swingTimingsPerLeg[leg]) && (time >= swingTimingsPerLeg[leg].back().start)) {
      // It is assumed that swing phase ends at infinity, so current time will be always close to SwingTiming.start
      swingPhasePerLeg[leg].phase = 0.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else {
      // Check if leg is in swing interval at current time
      auto it = std::find_if(swingTimingsPerLeg[leg].begin(), swingTimingsPerLeg[leg].end(),
                             [time](SwingTiming timing) { return (timing.start <= time) && (Time <= timing.end); });
      if (it == swingTimingsPerLeg[leg].end()) {
        // Leg is not swinging for current time
        swingPhasePerLeg[leg].phase = scalar_t(-1.0);
        swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
      } else {
        // Leg is swinging for current time
        const auto& currentSwingTiming = *it;
        swingPhasePerLeg[leg].phase = (time - currentSwingTiming.start) / (currentSwingTiming.end - currentSwingTiming.start);
        swingPhasePerLeg[leg].duration = currentSwingTiming.end - currentSwingTiming.start;
      }
    }
  }

  return swingPhasePerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<ContactTiming>> ExtractContactTimingsPerLeg(const bolza::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = ExtractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg) {
    contactTimingsPerLeg[leg] = ExtractContactTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return contactTimingsPerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<SwingTiming>> ExtractSwingTimingsPerLeg(const bolza::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = ExtractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < swingTimingsPerLeg.size(); ++leg) {
    swingTimingsPerLeg[leg] = ExtractSwingTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return swingTimingsPerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GetTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (HasEndTime(contactPhase) && contactPhase.end > currentTime) {
      return contactPhase.end;
    }
  }
  return TimingNaN();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GetTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (HasStartTime(contactPhase) && contactPhase.start > currentTime) {
      return contactPhase.start;
    }
  }
  return TimingNaN();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<ContactTiming> ExtractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<ContactTiming> contactTimings;
  contactTimings.reserve(1 + eventTimes.size() / 2);  // Approximate upper bound
  int currentPhase = 0;

  while (currentPhase < numPhases) {
    // Search where contact phase starts
    while (currentPhase < numPhases && !contactFlags[currentPhase]) {
      ++currentPhase;
    }
    if (currentPhase >= numPhases) {
      break;  // No more contact phases
    }

    // Register start of the contact phase
    const scalar_t startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the contact phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the contact phase
    const scalar_t endTime = (currentPhase + 1 >= numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    contactTimings.push_back({startTime, endTime});
    ++currentPhase;
  }
  return contactTimings;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<SwingTiming> ExtractSwingTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<SwingTiming> swingTimings;
  swingTimings.reserve(1 + eventTimes.size() / 2);  // Approximate upper bound
  int currentPhase = 0;

  while (currentPhase < numPhases) {
    // Search where swing phase starts
    while (currentPhase < numPhases && contactFlags[currentPhase]) {
      ++currentPhase;
    }
    if (currentPhase >= numPhases) {
      break;  // No more swing phases
    }

    // Register start of the swing phase
    const scalar_t startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the swing phase ends
    while (currentPhase + 1 < numPhases && !contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the contact phase
    const scalar_t endTime = (currentPhase + 1 >= numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    swingTimings.push_back({startTime, endTime});
    ++currentPhase;
  }
  return swingTimings;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>> ExtractContactFlags(const std::vector<size_t>& modeSequence) {
  const size_t numPhases = modeSequence.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = ModeNumber2StanceLeg(modeSequence[i]);
    for (size_t j = 0; j < contactFlagStock.size(); j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

}  // namespace legged_robot
}  // namespace bolza
