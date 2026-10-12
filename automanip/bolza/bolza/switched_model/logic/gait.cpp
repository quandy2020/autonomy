//
// Created by rgrandia on 15.03.20.
//

#include "bolza/switched_model/logic/gait.hpp"
#include "bolza/switched_model/core/motion_phase_definition.hpp"

#include <bolza/core/misc/display.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace switched_model {

bool IsValidGait(const Gait& gait) {
  bool validGait = true;
  validGait &= gait.duration > 0.0;
  validGait &= std::all_of(gait.eventPhases.begin(), gait.eventPhases.end(), [](scalar_t phase) { return 0.0 < phase && phase < 1.0; });
  validGait &= std::is_sorted(gait.eventPhases.begin(), gait.eventPhases.end());
  validGait &= gait.eventPhases.size() + 1 == gait.modeSequence.size();
  return validGait;
}

bool IsValidPhase(scalar_t phase) {
  return phase >= 0.0 && phase < 1.0;
}

scalar_t WrapPhase(scalar_t phase) {
  phase = std::fmod(phase, 1.0);
  if (phase < 0.0) {
    phase += 1.0;
  }
  return phase;
}

int GetModeIndexFromPhase(scalar_t phase, const Gait& gait) {
  assert(IsValidPhase(phase));
  assert(IsValidGait(gait));
  auto firstLargerValueIterator = std::upper_bound(gait.eventPhases.begin(), gait.eventPhases.end(), phase);
  return static_cast<int>(firstLargerValueIterator - gait.eventPhases.begin());
}

size_t GetModeFromPhase(scalar_t phase, const Gait& gait) {
  assert(IsValidPhase(phase));
  assert(IsValidGait(gait));
  return gait.modeSequence[GetModeIndexFromPhase(phase, gait)];
}

scalar_t TimeLeftInGait(scalar_t phase, const Gait& gait) {
  assert(IsValidPhase(phase));
  assert(IsValidGait(gait));
  return (1.0 - phase) * gait.duration;
}

scalar_t TimeLeftInMode(scalar_t phase, const Gait& gait) {
  assert(IsValidPhase(phase));
  assert(IsValidGait(gait));
  int modeIndex = GetModeIndexFromPhase(phase, gait);
  if (modeIndex < gait.eventPhases.size()) {
    return (gait.eventPhases[modeIndex] - phase) * gait.duration;
  } else {
    return TimeLeftInGait(phase, gait);
  }
}

std::ostream& operator<<(std::ostream& stream, const Gait& gait) {
  stream << "Duration:       " << gait.duration << "\n";
  stream << "Event phases:  {" << bolza::ToDelimitedString(gait.eventPhases) << "}\n";
  stream << "Mode sequence: {" << bolza::ToDelimitedString(gait.modeSequence) << "}\n";
  return stream;
}

}  // namespace switched_model
