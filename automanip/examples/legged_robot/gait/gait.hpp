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

#include <ostream>
#include <vector>

#include <bolza/core/types.hpp>

namespace bolza {
namespace legged_robot {

/**
 * A gait is a periodic mode schedule parameterized by a "phase" variable.
 *
 * The eventPhases only indicate switches of modes, i.e. phase = 0 and phase = 1 are not part of the eventPhases.
 * The number of modes is therefore number of phases + 1
 *
 * The conversion to time is regulated by a duration
 */
struct Gait {
  /** time for one gait cycle*/
  scalar_t duration;
  /** points in (0.0, 1.0) along the gait cycle where the contact mode changes, size N-1 */
  std::vector<scalar_t> eventPhases;
  /** sequence of contact modes, size: N */
  std::vector<size_t> modeSequence;
};

/**
 * isValidGait checks the following properties
 * - positive duration
 * - eventPhases are all in (0.0, 1.0)
 * - eventPhases are sorted
 * - the size of the modeSequences is 1 more than the eventPhases.
 */
bool IsValidGait(const Gait& gait);

/** Check is if the phase is in [0.0, 1.0) */
bool IsValidPhase(scalar_t phase);

/** Wraps a phase to [0.0, 1.0) */
scalar_t WrapPhase(scalar_t phase);

/** The modes are selected with a closed-open interval: [ ) */
int GetModeIndexFromPhase(scalar_t phase, const Gait& gait);

/** Gets the active mode from the phase variable */
size_t GetModeFromPhase(scalar_t phase, const Gait& gait);

/** Returns the time left in the gait based on the phase variable */
scalar_t TimeLeftInGait(scalar_t phase, const Gait& gait);

/** Returns the time left in the current based on the phase variable */
scalar_t TimeLeftInMode(scalar_t phase, const Gait& gait);

/** Print gait */
std::ostream& operator<<(std::ostream& stream, const Gait& gait);

}  // namespace legged_robot
}  // namespace bolza
