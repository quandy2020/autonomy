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

#pragma once

#include <bolza/core/reference/mode_schedule.hpp>

#include "examples/legged_robot/common/types.hpp"

namespace bolza {
namespace legged_robot {

struct LegPhase {
  scalar_t phase;
  scalar_t duration;
};

struct ContactTiming {
  scalar_t start;
  scalar_t end;
};

struct SwingTiming {
  scalar_t start;
  scalar_t end;
};

/**
 * @brief Get the contact phase for all legs.
 * If leg in contact, returns a value between 0.0 (at start of contact phase) and 1.0 (at end of contact phase).
 * If leg not in contact (i.e. in swing), returns -1.0.
 * If mode schedule starts with contact phase, returns 1.0 during this phase.
 * If mode schedule ends with contact phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Contact phases for all legs.
 */
feet_array_t<LegPhase> GetContactPhasePerLeg(scalar_t time, const bolza::ModeSchedule& modeSchedule);

/**
 * @brief Get the swing phase for all legs.
 * If leg in swing, returns a value between 0.0 (at start of swing phase) and 1.0 (at end of swing phase).
 * If leg not in swing (i.e. in contact), returns -1.0.
 * If mode schedule starts with swing phase, returns 1.0 during this phase.
 * If mode schedule ends with swing phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Swing phases for all legs.
 */
feet_array_t<LegPhase> GetSwingPhasePerLeg(scalar_t time, const bolza::ModeSchedule& modeSchedule);

/** Extracts the contact timings for all legs from a modeSchedule */
feet_array_t<std::vector<ContactTiming>> ExtractContactTimingsPerLeg(const bolza::ModeSchedule& modeSchedule);

/** Extracts the swing timings for all legs from a modeSchedule */
feet_array_t<std::vector<SwingTiming>> ExtractSwingTimingsPerLeg(const bolza::ModeSchedule& modeSchedule);

/** Returns time of the next lift off. Returns nan if leg is not lifting off */
scalar_t GetTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/** Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down */
scalar_t GetTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/**
 * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no contact phases
 */
std::vector<ContactTiming> ExtractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Get {startTime, endTime} for all swing phases. Contact phases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no swing phases
 */
std::vector<SwingTiming> ExtractSwingTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param modeSequence : Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> ExtractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace legged_robot
}  // namespace bolza
