/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_TRIGONOMETRIC_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_TRIGONOMETRIC_HPP_

#include <cmath>

#include <opencv2/core/fast_math.hpp>

namespace autonomy::localization::atlas {
namespace util {

static constexpr float _PI = 3.14159265358979f;
static constexpr float _PI_2 = _PI / 2.0f;
static constexpr float _TWO_PI = 2.0f * _PI;
static constexpr float _INV_TWO_PI = 1.0f / _TWO_PI;
static constexpr float _THREE_PI_2 = 3.0f * _PI_2;

inline float _cos(float v) {
    constexpr float c1 = 0.99940307f;
    constexpr float c2 = -0.49558072f;
    constexpr float c3 = 0.03679168f;

    const float v2 = v * v;
    return c1 + v2 * (c2 + c3 * v2);
}

inline float cos(float v) {
    v = v - cvFloor(v * _INV_TWO_PI) * _TWO_PI;
    v = (0.0f < v) ? v : -v;

    if (v < _PI_2) {
        return _cos(v);
    }
    else if (v < _PI) {
        return -_cos(_PI - v);
    }
    else if (v < _THREE_PI_2) {
        return -_cos(v - _PI);
    }
    else {
        return _cos(_TWO_PI - v);
    }
}

inline float sin(float v) {
    return autonomy::localization::atlas::util::cos(_PI_2 - v);
}

} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_TRIGONOMETRIC_HPP_
