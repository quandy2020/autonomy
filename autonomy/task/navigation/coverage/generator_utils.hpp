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

/**
 * @file generator_utils.hpp
 * @brief Numeric validation shared by coverage stages.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_GENERATOR_UTILS_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_GENERATOR_UTILS_HPP_

#include <cmath>
#include <limits>
#include <set>

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Rejects a non-positive or non-finite length or speed.
 * @param[in] value Value taken from the request or the configuration.
 * @param[in] name Field name used in the error message.
 * @throws RequestException when @p value is not a positive finite number.
 */
inline void Positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0)
    throw RequestException(std::string(name) + " must be positive and finite");
}
/**
 * @brief Rejects a non-finite angle or coordinate.
 * @param[in] value Angle in radians or a coordinate in metres.
 * @throws RequestException when @p value is not finite.
 */
inline void Angle(double value) {
  if (!std::isfinite(value)) throw RequestException("Angle must be finite");
}

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_GENERATOR_UTILS_HPP_
