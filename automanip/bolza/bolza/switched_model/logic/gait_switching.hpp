//
// Created by rgrandia on 15.03.20.
//

#pragma once

/**
 * @file gait_switching.hpp
 * @brief Selects the next gait from iterators. Repeating the final gait when the final gait is reached.
 */


#include <bolza/core/reference/mode_schedule.hpp>

#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/logic/gait.hpp"

namespace switched_model {

/**
 * Selects the next gait from iterators. Repeating the final gait when the final gait is reached.
 * @tparam GaitIt : Gait iterator
 * @param [in] currentGait: iterator pointing to the current gait
 * @param [in] pastTheEndGait: iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return next gait in the sequence
 */
template <typename GaitIt>
/**
 * @brief Next Gait.
 * @param [in] currentGait: Current Gait.
 * @param [in] pastTheEndGait: Past The End Gait.
 * @return Result value.
 */
GaitIt NextGait(GaitIt currentGait, GaitIt pastTheEndGait) {
  GaitIt nextGait = currentGait + 1;
  if (nextGait == pastTheEndGait) {
    return currentGait;
  } else {
    return nextGait;
  }
}

/**
 * Recursively progresses to the next gait in a sequence while keeping track of the phase variable.
 * When the final gait is reached, it is repeated as long as required by the specified dt.
 *
 * @tparam GaitIt : Gait iterator
 * @param [in] phase: phase variable in the current gait.
 * @param [in] dt: time to progress in the gait sequence
 * @param [in] currentGait: iterator pointing to the current gait
 * @param [in] pastTheEndGait: iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return {phase in new gait, iterator to newly active gait.
 */
template <typename GaitIt>
/**
 * @brief Advance Phase.
 * @param [in] phase: Phase.
 * @param [in] dt: Dt.
 * @param [in] currentGait: Current Gait.
 * @param [in] pastTheEndGait: Past The End Gait.
 * @return Result value.
 */
std::pair<scalar_t, GaitIt> AdvancePhase(scalar_t phase, scalar_t dt, GaitIt currentGait, GaitIt pastTheEndGait) {
  assert(IsValidPhase(phase));
  assert(dt >= 0);

  // Phase change within current gait
  scalar_t dphase = dt / currentGait->duration;

  if (phase + dphase < 1.0) {  // Advance within current gait
    phase += dphase;
    return {phase, currentGait};
  } else {  // Advance to next gait
    const scalar_t dtRemaining = std::max(dt - TimeLeftInGait(phase, *currentGait), 0.0);
    const GaitIt nexGait = NextGait(currentGait, pastTheEndGait);
    // Recurse by setting the phase to the beginning of the next phase
    /**
     * @brief Advance Phase.
     * @param [in] dtRemaining: Dt Remaining.
     * @param [in] nexGait: Nex Gait.
     * @param [in] pastTheEndGait: Past The End Gait.
     * @return Result value.
     */
    return AdvancePhase(0.0, dtRemaining, nexGait, pastTheEndGait);
  }
}

template <typename GaitIt>
/**
 * @brief Get Mode Schedule.
 * @param [in] phase: Phase.
 * @param [in] t0: T0.
 * @param [in] timeHorizon: Time Horizon.
 * @param [in] currentGait: Current Gait.
 * @param [in] pastTheEndGait: Past The End Gait.
 * @return Result value.
 */
bolza::ModeSchedule GetModeSchedule(scalar_t phase, scalar_t t0, scalar_t timeHorizon, GaitIt currentGait, GaitIt pastTheEndGait) {
  assert(IsValidPhase(phase));

  // Initialize with the current mode
  std::vector<scalar_t> evenTimes = {};
  std::vector<size_t> modeSchedule = {GetModeFromPhase(phase, *currentGait)};

  const scalar_t tend = t0 + timeHorizon;
  scalar_t t = t0;
  while (t < tend) {
    scalar_t dt = TimeLeftInMode(phase, *currentGait);
    t += dt;
    if (t < tend) {  // Next event is within horizon: Add the event time and advance the phase to that event
      /**
       * @brief Tie.
       * @param [in] phase: Phase.
       * @return Result value.
       */
      std::tie(phase, currentGait) = AdvancePhase(phase, dt, currentGait, pastTheEndGait);
      size_t mode = GetModeFromPhase(phase, *currentGait);
      if (mode != modeSchedule.back()) {  // Only add the mode if it is a switch w.r.t the last mode.
        /**
         * @brief Push back.
         * @param [in] t: T.
         * @return Result value.
         */
        evenTimes.push_back(t);
        /**
         * @brief Push back.
         * @param [in] mode: Mode.
         * @return Result value.
         */
        modeSchedule.push_back(mode);
      }
    }
  }
  return {evenTimes, modeSchedule};
}

}  // namespace switched_model
