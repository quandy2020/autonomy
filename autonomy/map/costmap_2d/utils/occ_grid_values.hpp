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

#include <cmath>
#include <cstdint>

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

struct Rgba8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

/** Matches map_io Scale mode and Cartographer CreateOccupancyGridMsg coloring. */
inline Rgba8 OccupancyCellToRgba(int8_t cell) {
    static constexpr uint8_t kUnknownGray = 205;
    static constexpr uint8_t kFreeGray = 254;

    if (cell < 0 || cell > OCC_GRID_OCCUPIED) {
        return {kUnknownGray, kUnknownGray, kUnknownGray, 255};
    }
    if (cell == OCC_GRID_FREE) {
        return {kFreeGray, kFreeGray, kFreeGray, 255};
    }
    if (cell >= OCC_GRID_OCCUPIED) {
        return {0, 0, 0, 255};
    }
    const uint8_t shade = static_cast<uint8_t>(
        std::lround((100.0 - static_cast<double>(cell)) * 255.0 / 100.0));
    return {shade, shade, shade, 255};
}

}  // namespace utils
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
