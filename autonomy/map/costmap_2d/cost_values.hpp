/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

/** Provides a mapping for often used cost values */
namespace autonomy {
namespace map {
namespace costmap_2d {

static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;

struct CostRgba8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

/** Inverse of StaticLayer occupancy→cost scaling (lethal_threshold default 100). */
inline int8_t CostCellToOccupancyEquivalent(
    unsigned char cost,
    int8_t lethal_threshold = utils::OCC_GRID_OCCUPIED) {
    if (cost == NO_INFORMATION) {
        return utils::OCC_GRID_UNKNOWN;
    }
    if (cost == FREE_SPACE) {
        return utils::OCC_GRID_FREE;
    }
    if (cost >= LETHAL_OBSTACLE) {
        return utils::OCC_GRID_OCCUPIED;
    }
    const int occ = static_cast<int>(std::lround(
        static_cast<double>(cost) * static_cast<double>(lethal_threshold) /
        static_cast<double>(LETHAL_OBSTACLE)));
    return static_cast<int8_t>(
        std::clamp(occ, 1, static_cast<int>(utils::OCC_GRID_OCCUPIED) - 1));
}

/** Same grayscale palette as ROS OccupancyGrid / map.pgm. */
inline CostRgba8 CostCellToRgba(unsigned char cost) {
    const utils::Rgba8 rgba =
        utils::OccupancyCellToRgba(CostCellToOccupancyEquivalent(cost));
    return {rgba.r, rgba.g, rgba.b, rgba.a};
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
