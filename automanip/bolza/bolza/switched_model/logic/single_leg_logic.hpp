//
// Created by rgrandia on 27.04.20.
//

#pragma once

/**
 * @file single_leg_logic.hpp
 * @brief Extracts the contact timings for all legs from a modeSchedule.
 */


#include <bolza/core/reference/mode_schedule.hpp>

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

struct ContactTiming {
  scalar_t start;
  scalar_t end;
};

/**
 * @brief Timing Na N.
 * @return Result value.
 */
inline scalar_t TimingNaN() {
  /**
   * @brief Quiet Na N.
   * @return Result value.
   */
  return std::numeric_limits<scalar_t>::quiet_NaN();
}

/**
 * @brief Has Start Time.
 * @param [in] timing: Timing.
 * @return True on success.
 */
inline bool HasStartTime(const ContactTiming& timing) {
  /**
   * @brief Isnan.
   * @param [in] start: Start.
   * @return Result value.
   */
  return !std::isnan(timing.start);
}
/**
 * @brief Has End Time.
 * @param [in] timing: Timing.
 * @return True on success.
 */
inline bool HasEndTime(const ContactTiming& timing) {
  /**
   * @brief Isnan.
   * @param [in] end: End.
   * @return Result value.
   */
  return !std::isnan(timing.end);
}
/**
 * @brief Starts With Swing Phase.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool StartsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Empty.
   * @return Result value.
   */
  return timings.empty() || HasStartTime(timings.front());
}
/**
 * @brief Starts With Stance Phase.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool StartsWithStancePhase(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Starts With Swing Phase.
   * @param [in] timings: Timings.
   * @return Result value.
   */
  return !StartsWithSwingPhase(timings);
}
/**
 * @brief Ends With Swing Phase.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool EndsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Empty.
   * @return Result value.
   */
  return timings.empty() || HasEndTime(timings.back());
}
/**
 * @brief Ends With Stance Phase.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool EndsWithStancePhase(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Ends With Swing Phase.
   * @param [in] timings: Timings.
   * @return Result value.
   */
  return !EndsWithSwingPhase(timings);
}

/**
 * @brief Touches Down At Least Once.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool TouchesDownAtLeastOnce(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Any of.
   * @return Result value.
   */
  return std::any_of(timings.begin(), timings.end(), [](const ContactTiming& timing) { return HasStartTime(timing); });
}

/**
 * @brief Lifts Off At Least Once.
 * @param [in] timings: Timings.
 * @return True on success.
 */
inline bool LiftsOffAtLeastOnce(const std::vector<ContactTiming>& timings) {
  /**
   * @brief Empty.
   * @return Result value.
   */
  return !timings.empty() && HasEndTime(timings.front());
}

/**
 * @brief Extracts the contact timings for all legs from a modeSchedule.
 * @param [in] modeSchedule: Mode Schedule.
 * @return Result value.
 */
feet_array_t<std::vector<ContactTiming>> ExtractContactTimingsPerLeg(const bolza::ModeSchedule& modeSchedule);

/**
 * @brief Returns time of the next lift off. Returns nan if leg is not lifting off.
 * @param [in] currentTime: Current Time.
 * @param [in] contactTimings: Contact Timings.
 * @return Result value.
 */
scalar_t GetTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/**
 * @brief Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down.
 * @param [in] currentTime: Current Time.
 * @param [in] contactTimings: Contact Timings.
 * @return Result value.
 */
scalar_t GetTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/**
 * @brief Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1].
 *
 * times are NaN if they cannot be identified at the boundaries
 *
 * Vector is empty if there are no contact phases
 * @param [in] eventTimes: Event Times.
 * @param [in] contactFlags: Contact Flags.
 * @return Result value.
 */
std::vector<ContactTiming> ExtractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param [in] modeSequence: Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> ExtractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace switched_model
