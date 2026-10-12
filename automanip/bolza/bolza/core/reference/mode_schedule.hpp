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

/**
 * @file mode_schedule.hpp
 * @brief Defines a sequence of N modes, separated by N-1 event times.
 */


#include <ostream>
#include <vector>

#include <bolza/core/types.hpp>

namespace bolza {

/**
 * Defines a sequence of N modes, separated by N-1 event times
 */
struct ModeSchedule {
  /**
   * Default constructor.
   */
  ModeSchedule() : ModeSchedule(std::vector<scalar_t>{}, std::vector<size_t>{0}) {}

  /**
   * Constructor for a ModeSchedule. The number of modes must be greater than zero (N > 0)
   * @param [in] eventTimesInput: event times of size N - 1
   * @param [in] modeSequenceInput: mode sequence of size N
   */
  ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput);

    /**
   * @brief.
   *
   * Returns the mode based on the query time.
   *
   * Events are counted as follows:
   *
   * ------ | ------ | ------ | ...  ------ | ------
   *
   * t[0]     t[1]     t[2]        t[n-1]
   *
   * mode: m[0]    m[1]     m[2] ...     m[n-1]    m[n]
   *
   * If time equal to a switch time is requested, the lower count is taken
   *
   * @param [in] time: The inquiry time.
   *
   * @return the associated mode for the input time.
   *
   * /
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t ModeAtTime(scalar_t time) const;

  /** Clears modeSchedule */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    eventTimes.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    modeSequence.clear();
  }

  std::vector<scalar_t> eventTimes;  // event times of size N - 1
  std::vector<size_t> modeSequence;  // mode sequence of size N
};

/**
 * @brief Exchanges the given values. 
 * @param [in,out] lh: Lh.
 * @param [in,out] rh: Rh.
 */
void swap(ModeSchedule& lh, ModeSchedule& rh);

/** Inserts modeSchedule into the output stream. */
std::ostream& operator<<(std::ostream& stream, const ModeSchedule& modeSchedule);

/**
 * Gets the number of events that have preceded the given event time.
 *
 * @param [in] timeTrajectory: A trajectory of timestamps.
 * @param [in] postEventIndices: An array of post-event time index in the time trajectory.
 * @param [in] eventTime: The requested event time.
 * @return The number of events that have preceded the given event time.
 */
size_t GetNumberOfPrecedingEvents(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, scalar_t eventTime);

/**
 * Finds the intersection of the requested period i.e., timePeriod, to the time interval of the modes that timeTrajectory
 * covers, i.e., extendable interval.
 *
 * Case 1: non-empty interval
 * eventTimes       s0             s2         s2          s3
 * ------------------------|--------------|----------|-----------|-------
 * timeTrajectory ---------------****************------------------------
 * Extendable interval ----+++++++++++++++++++++++++++-------------------
 * Requested timePeriod ----------------------[           ]--------------
 * Output ------------------------------------[******]-------------------
 *
 * Case 2: empty interval
 * eventTimes       s0             s2         s2          s3
 * ------------------------|--------------|----------|-----------|-------
 * timeTrajectory ---------------******************----------------------
 * Extendable interval ----+++++++++++++++++++++++++++-------------------
 * Requested timePeriod -----------------------------------[         ]---
 * Output ------------------------------------------------][-------------
 *
 * @param [in] timeTrajectory: Timestamp of the data to be interpolated.
 * @param [in] eventTimes: The event times array.
 * @param [in] timePeriod: The requested time period for possible interpolation.
 * @return The truncated time interval in which the interpolation is valid. In the case that the interpolation is not allowed it
 * returns and interval of negative length i.e., (timePeriod.first, timePeriod.first - 1e-4).
 */
std::pair<scalar_t, scalar_t> FindIntersectionToExtendableInterval(const scalar_array_t& timeTrajectory, const scalar_array_t& eventTimes,
                                                                   const std::pair<scalar_t, scalar_t>& timePeriod);

}  // namespace bolza
