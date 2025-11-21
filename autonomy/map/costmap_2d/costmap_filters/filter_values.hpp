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

#pragma once

#include "autonomy/common/port.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/** Types of costmap filter */
static constexpr uint8_t KEEPOUT_FILTER = 0;
static constexpr uint8_t SPEED_FILTER_PERCENT = 1;
static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2;
static constexpr uint8_t BINARY_FILTER = 3;

/** Default values for base and multiplier */
static constexpr double BASE_DEFAULT = 0.0;
static constexpr double MULTIPLIER_DEFAULT = 1.0;

/** Speed filter constants */
static constexpr int8_t SPEED_MASK_UNKNOWN = -1;
static constexpr int8_t SPEED_MASK_NO_LIMIT = 0;
static constexpr double NO_SPEED_LIMIT = 0.0;

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
