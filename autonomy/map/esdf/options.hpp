/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file options.hpp
 * @brief Dense-array FIESTA map geometry and occupancy-fusion parameters.
 */

#pragma once

#include <Eigen/Core>

namespace autonomy {
namespace map {
namespace esdf {

/**
 * @struct autonomy::map::esdf::Options
 * @brief Fixed-grid extents and log-odds occupancy knobs (no ROS).
 *
 * Default occupancy_probability is above a single hit_probability update; a
 * voxel typically needs two occupied observations (each followed by
 * UpdateOccupancy) before it is occupied.
 */
struct Options {
    /**
     * @brief World-frame corner of voxel (0, 0, 0).
     *
     * Units are metres. Voxel (i, j, k) covers
     * [origin + i * resolution, origin + (i + 1) * resolution).
     */
    Eigen::Vector3d origin{-5.0, -5.0, -1.0};

    /**
     * @brief Axis-aligned map extent from @ref origin.
     *
     * Units are metres. Grid size along each axis is
     * ceil(map_size[axis] / resolution).
     */
    Eigen::Vector3d map_size{10.0, 10.0, 2.0};

    /**
     * @brief Cubic voxel edge length.
     *
     * Units are metres. Must be positive. Inverse resolution is cached on
     * EsdfMap construction.
     */
    double resolution{0.1};

    /**
     * @brief Occupied-observation probability used as a logit increment.
     *
     * Typical range (0, 1). Converted with log(p / (1 - p)) in SetProbabilities.
     */
    double hit_probability{0.70};

    /**
     * @brief Free-observation probability used as a logit decrement.
     *
     * Typical range (0, 1). A miss is applied when hits are not the majority
     * of observations on that voxel in the current UpdateOccupancy cycle.
     */
    double miss_probability{0.35};

    /**
     * @brief Lower occupancy-probability clamp before converting to log-odds.
     *
     * Occupancy_buffer_ is clamped to [logit(min_probability),
     * logit(max_probability)] after each fusion step.
     */
    double min_probability{0.12};

    /**
     * @brief Upper occupancy-probability clamp before converting to log-odds.
     *
     * See @ref min_probability. Values near 1.0 make occupied cells saturate
     * quickly and require more misses to free.
     */
    double max_probability{0.97};

    /**
     * @brief Occupied if fused occupancy probability exceeds this threshold.
     *
     * Compared in log-odds space. With the defaults, one hit does not occupy;
     * two hits (each followed by UpdateOccupancy) typically do.
     */
    double occupancy_probability{0.80};

    /**
     * @brief Rays shorter than this are ignored by IntegrateRay.
     *
     * Units are metres. Set to 0 to accept arbitrarily short rays (tests).
     */
    double min_ray_length{0.5};

    /**
     * @brief Rays longer than this are clamped and marked free only.
     *
     * Units are metres. The endpoint is moved onto the sphere of this radius
     * around the sensor origin; no occupied hit is recorded.
     */
    double max_ray_length{5.0};
};

}  // namespace esdf
}  // namespace map
}  // namespace autonomy
