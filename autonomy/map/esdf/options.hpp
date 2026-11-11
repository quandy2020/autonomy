/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Port of FIESTA (hlx1996, MIT) occupancy/ESDF parameters without ROS.
 */

#pragma once

#include <Eigen/Core>

namespace autonomy {
namespace map {
namespace esdf {

/** @brief Dense-array FIESTA map geometry and occupancy fusion knobs. */
struct Options {
    /** @brief World-frame corner of voxel (0,0,0). */
    Eigen::Vector3d origin{-5.0, -5.0, -1.0};
    /** @brief Axis-aligned extent from origin (metres). */
    Eigen::Vector3d map_size{10.0, 10.0, 2.0};
    /** @brief Cubic voxel size (metres). */
    double resolution{0.1};
    /** @brief Occupied observation probability. */
    double p_hit{0.70};
    /** @brief Free observation probability. */
    double p_miss{0.35};
    /** @brief Log-odds lower clamp. */
    double p_min{0.12};
    /** @brief Log-odds upper clamp. */
    double p_max{0.97};
    /** @brief Occupied if probability exceeds this. */
    double p_occ{0.80};
    /** @brief Rays shorter than this are ignored (metres). */
    double min_ray_length{0.5};
    /** @brief Rays longer than this are clamped and marked free. */
    double max_ray_length{5.0};
};

}  // namespace esdf
}  // namespace map
}  // namespace autonomy
