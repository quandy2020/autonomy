//
// Created by rgrandia on 15.03.20.
//

#pragma once

/**
 * @file gait.hpp
 * @brief A gait is a periodic mode schedule parameterized by a "phase" variable.
 */


#include <ostream>
#include <vector>

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

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
 * @brief isValidGait checks the following properties.
 *
 * - positive duration
 *
 * - eventPhases are all in (0.0, 1.0)
 *
 * - eventPhases are sorted
 *
 * - the size of the modeSequences is 1 more than the eventPhases.
 * @param [in] gait: Gait.
 * @return True on success.
 */
bool IsValidGait(const Gait& gait);

/**
 * @brief Check is if the phase is in [0.0, 1.0).
 * @param [in] phase: Phase.
 * @return True on success.
 */
bool IsValidPhase(scalar_t phase);

/**
 * @brief Wraps a phase to [0.0, 1.0).
 * @param [in] phase: Phase.
 * @return Result value.
 */
scalar_t WrapPhase(scalar_t phase);

/**
 * @brief The modes are selected with a closed-open interval: [ ).
 * @param [in] phase: Phase.
 * @param [in] gait: Gait.
 * @return Result value.
 */
int GetModeIndexFromPhase(scalar_t phase, const Gait& gait);

/**
 * @brief Gets the active mode from the phase variable.
 * @param [in] phase: Phase.
 * @param [in] gait: Gait.
 * @return Result value.
 */
size_t GetModeFromPhase(scalar_t phase, const Gait& gait);

/**
 * @brief Returns the time left in the gait based on the phase variable.
 * @param [in] phase: Phase.
 * @param [in] gait: Gait.
 * @return Result value.
 */
scalar_t TimeLeftInGait(scalar_t phase, const Gait& gait);

/**
 * @brief Returns the time left in the current based on the phase variable.
 * @param [in] phase: Phase.
 * @param [in] gait: Gait.
 * @return Result value.
 */
scalar_t TimeLeftInMode(scalar_t phase, const Gait& gait);

/** Print gait */
std::ostream& operator<<(std::ostream& stream, const Gait& gait);

}  // namespace switched_model
