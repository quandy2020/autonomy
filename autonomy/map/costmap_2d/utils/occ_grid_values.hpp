/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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
namespace utils {
 
/**
 * @brief OccupancyGrid data constants
 */
static constexpr int8 OCC_GRID_UNKNOWN = -1;
static constexpr int8 OCC_GRID_FREE = 0;
static constexpr int8 OCC_GRID_OCCUPIED = 100;

}  // namespace utils
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
