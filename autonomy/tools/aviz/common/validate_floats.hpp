/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <limits>

/**
 * @brief Float validation utilities
 * Float validation helpers
 *
 * Provides functions to validate that floating point values are valid (not NaN, not Inf)
 */
namespace aviz {
namespace common {

/**
 * @brief Check if a float value is valid (not NaN, not Inf)
 */
inline bool validateFloat(float value) {
    return std::isfinite(value) && !std::isnan(value);
}

/**
 * @brief Check if a double value is valid (not NaN, not Inf)
 */
inline bool validateFloat(double value) {
    return std::isfinite(value) && !std::isnan(value);
}

/**
 * @brief Validate a 3D vector (all components must be valid)
 */
template <typename T>
inline bool validateVector3(T x, T y, T z) {
    return validateFloat(x) && validateFloat(y) && validateFloat(z);
}

/**
 * @brief Validate a quaternion (all components must be valid)
 */
template <typename T>
inline bool validateQuaternion(T x, T y, T z, T w) {
    return validateFloat(x) && validateFloat(y) && validateFloat(z) && validateFloat(w);
}

/**
 * @brief Validate a pose (position and orientation must be valid)
 */
template <typename T>
inline bool validatePose(T px, T py, T pz, T ox, T oy, T oz, T ow) {
    return validateVector3(px, py, pz) && validateQuaternion(ox, oy, oz, ow);
}

}  // namespace common
}  // namespace aviz
