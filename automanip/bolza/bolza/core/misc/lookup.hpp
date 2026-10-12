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
 * @file lookup.hpp
 * @brief finds the index of an element in a sorted dataArray which is equal to Value (epsilon distance).
 */


#include <algorithm>
#include <sstream>
#include <vector>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/misc/numerics.hpp>

namespace bolza {
namespace lookup {

/**
 * finds the index of an element in a sorted dataArray which is equal to Value (epsilon distance)
 *
 * @tparam SCALAR : scalar type
 * @param [in] dataArray: data array
 * @param [in] value: enquiry value
 * @return index
 */
template <typename SCALAR = double>
/**
 * @brief Find First Index Within Tol.
 * @param [in] dataArray: Data Array.
 * @param [in] value: Value.
 * @param [in] eps: Eps.
 * @return Result value.
 */
size_t FindFirstIndexWithinTol(const std::vector<SCALAR>& dataArray, SCALAR value, SCALAR eps = numeric_traits::WeakEpsilon<SCALAR>()) {
  // Search for a match by linearly traversing the data, returning first match
  for (size_t i = 0; /**
  * @brief Size.
  * @return Result value.
  */
 i < dataArray.size(); i++) {
    if (std::abs(dataArray[i] - value) < eps) {
      return i;
    }
  }

  // If we reach here, no match was found
  std::stringstream msg;
  msg << "[findFirstIndexWithinTol] Value not found within tolerance, with \n\t dataArray: ";
  for (const auto d : dataArray) {
    msg << d << ", ";
  }
  msg << "\n\t value: " << Value << "\n\t epsilon: " << Eps << std::endl;
  /**
   * @brief Runtime error.
   * @return Result value.
   */
  throw std::runtime_error(msg.str());
}

/**
 *  Find index into a sorted time Array
 *
 *  Indices are counted as follows:
 *          ------ | ----- | ---  ... ---    | -----
 *                t0     t1              t(n-1)
 *  Index     0        1      2   ...  (n-1)    n
 *
 *  Corner cases:
 *     - If time equal to a time in the timeArray is requested, the lower index is taken (e.g. t = t1 -> index = 1)
 *     - If multiple times in the timeArray are equal, the index before the first occurrence is taken.
 *       for example: if t1 = t2 = t3  and the requested time t <= t3 -> index = 1
 *
 *
 * @tparam SCALAR : numerical type of time
 * @param [in] timeArray: sorted time array to perform the lookup in
 * @param [in] time: enquiry time
 * @return index between [0, size(timeArray)]
 */
template <typename SCALAR = double>
/**
 * @brief Find Index In Time Array.
 * @param [in] timeArray: Time Array.
 * @param [in] time: Time.
 * @return Result value.
 */
int FindIndexInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  /**
   * @brief Lower bound.
   * @param [in] time: Time.
   * @return Result value.
   */
  auto firstLargerValueIterator = std::lower_bound(timeArray.begin(), timeArray.end(), time);
  /**
   * @brief Begin.
   * @return Result value.
   */
  return static_cast<int>(firstLargerValueIterator - timeArray.begin());
}

/**
 *  Find interval into a sorted time Array
 *
 *  Intervals are counted as follows:
 *           ------ | ----- | ---  ... ---    | -----
 *                 t0     t1              t(n-1)
 *  Interval  -1       0      1   ...  (n-2)    (n-1)
 *
 *  Corner cases are handled as in findIndexInTimeArray
 *
 * @tparam SCALAR : numerical type of time
 * @param [in] timeArray: sorted time array to perform the lookup in
 * @param [in] time: enquiry time
 * @return interval between [-1, size(timeArray)-1]
 */
template <typename SCALAR = double>
/**
 * @brief Find Interval In Time Array.
 * @param [in] timeArray: Time Array.
 * @param [in] time: Time.
 * @return Result value.
 */
int FindIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  if (!timeArray.empty()) {
    return FindIndexInTimeArray(timeArray, time) - 1;
  } else {
    return 0;
  }
}

/**
 * Same as findIntervalInTimeArray except for 1 rule:
 * if t = t0, a 0 is returned instead of -1
 * This means that any query t_front <= t <= t_back gets assigned to a partition [0, size -2]
 *
 * @tparam SCALAR : scalar type
 * @param [in] timeArray: sorted time array to perform the lookup in
 * @param [in] time: enquiry time
 * @return partition between [-1, size(timeArray)-1]
 */
template <typename SCALAR = double>
/**
 * @brief Find Active Interval In Time Array.
 * @param [in] timeArray: Time Array.
 * @param [in] time: Time.
 * @return Result value.
 */
int FindActiveIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  if (!timeArray.empty() && !numerics::almost_eq(time, timeArray.front())) {
    /**
     * @brief Find Interval In Time Array.
     * @param [in] timeArray: Time Array.
     * @param [in] time: Time.
     * @return Result value.
     */
    return FindIntervalInTimeArray(timeArray, time);
  } else {  // t = t0
    return 0;
  }
}

/**
 * Wraps findActiveIntervalInTimeArray with bound check
 * throws an error if time before of after the end is selected.
 * Also throws on timeArrays that are empty or with 1 element only.
 *
 * @tparam SCALAR : scalar type
 * @param [in] timeArray: sorted time array to perform the lookup in
 * @param [in] time: enquiry time
 * @return partition between [0, size(timeArray)-2]
 */
template <typename SCALAR = double>
/**
 * @brief Find Bounded Active Interval In Time Array.
 * @param [in] timeArray: Time Array.
 * @param [in] time: Time.
 * @return Result value.
 */
int FindBoundedActiveIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  /**
   * @brief Find Active Interval In Time Array.
   * @param [in] timeArray: Time Array.
   * @param [in] time: Time.
   * @return Result value.
   */
  auto partition = FindActiveIntervalInTimeArray(timeArray, time);

  if (timeArray.empty()) {
    std::string mesg = "[findBoundedActiveIntervalInTimeArray] Time array is empty";
    /**
     * @brief Runtime error.
     * @param [in] mesg: Mesg.
     * @return Result value.
     */
    throw std::runtime_error(mesg);
  }

  if (partition < 0) {
    std::string mesg =
        "[findBoundedActiveIntervalInTimeArray] Given time is less than the start time (i.e. givenTime < timeArray.front()): " +
        /**
         * @brief To string.
         * @return Result value.
         */
        std::to_string(time) + " < " + std::to_string(timeArray.front());
    /**
     * @brief Runtime error.
     * @param [in] mesg: Mesg.
     * @return Result value.
     */
    throw std::runtime_error(mesg);
  }

  if (partition >= timeArray.size() - 1) {
    std::string mesg =
        "[findBoundedActiveIntervalInTimeArray] Given time is greater than the final time (i.e. timeArray.back() < givenTime): " +
        /**
         * @brief To string.
         * @param [in] time: Time.
         * @return Result value.
         */
        std::to_string(timeArray.back()) + " < " + std::to_string(time);
    /**
     * @brief Runtime error.
     * @param [in] mesg: Mesg.
     * @return Result value.
     */
    throw std::runtime_error(mesg);
  }

  return partition;
}

}  // namespace lookup
}  // namespace bolza
