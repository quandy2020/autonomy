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
 * @file trapezoidal_integration.hpp
 * @brief Compute the trapezoidal integration of a trajectory of VALUE_T given the time stamp timeTrajectory and initial value initialValue.
 */


#include <cassert>
#include <cstddef>
#include <vector>

namespace bolza {

/**
 * Compute the trapezoidal integration of a trajectory of VALUE_T given the time stamp timeTrajectory and initial value initialValue.
 *
 * @note It requires that the VALUE_T has overwrite operator+(VALUE_T, VALUE_T) and define VALUE_T::operator*(SCALAR_T)
 */
template <typename SCALAR_T, typename VALUE_T>
/**
 * @brief Trapezoidal Integration.
 * @param [in] timeTrajectory: Time Trajectory.
 * @param [in] valueTrajectory: Value Trajectory.
 * @param [in] initialValue: Initial Value.
 * @return Result value.
 */
VALUE_T TrapezoidalIntegration(const std::vector<SCALAR_T>& timeTrajectory, const std::vector<VALUE_T>& valueTrajectory,
                               VALUE_T initialValue) {
  /**
   * @brief Assert.
   */
  assert(timeTrajectory.size() == valueTrajectory.size());

  if (timeTrajectory.size() < 2) {
    return initialValue;
  }

  for (std::size_t k = 1; /**
  * @brief Size.
  * @return Result value.
  */
 k < timeTrajectory.size(); k++) {
    auto temp = valueTrajectory[k - 1] + valueTrajectory[k];
    temp *= (0.5 * (timeTrajectory[k] - timeTrajectory[k - 1]));
    initialValue += temp;
  }  // end of k loop

  return initialValue;
}

}  // namespace bolza
